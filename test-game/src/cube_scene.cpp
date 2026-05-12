#include "cube_scene.hh"
#include "madnight.hh"

#include "psyqo/advancedpad.hh"
#include "psyqo/gte-kernels.hh"
#include "psyqo/gte-registers.hh"
#include "psyqo/soft-math.hh"

using namespace psyqo::fixed_point_literals;
using namespace psyqo::trig_literals;

namespace {

struct Face {
    uint8_t vertices[4];
    psyqo::Color color;
};

struct Quad3D {
    psyqo::Vec3 v[4];
    psyqo::Color color;
};

constexpr psyqo::Vec3 c_cubeVertices[8] = {
    {.x = -0.05, .y = -0.05, .z = -0.05}, {.x = 0.05, .y = -0.05, .z = -0.05},
    {.x = -0.05, .y = 0.05,  .z = -0.05}, {.x = 0.05, .y = 0.05,  .z = -0.05},
    {.x = -0.05, .y = -0.05, .z = 0.05},  {.x = 0.05, .y = -0.05, .z = 0.05},
    {.x = -0.05, .y = 0.05,  .z = 0.05},  {.x = 0.05, .y = 0.05,  .z = 0.05},
};

constexpr Face c_cubeFaces[6] = {
    {.vertices = {0, 1, 2, 3}, .color = {0, 0, 255}},
    {.vertices = {6, 7, 4, 5}, .color = {0, 255, 0}},
    {.vertices = {4, 5, 0, 1}, .color = {0, 255, 255}},
    {.vertices = {7, 6, 3, 2}, .color = {255, 0, 0}},
    {.vertices = {6, 4, 2, 0}, .color = {255, 0, 255}},
    {.vertices = {5, 7, 1, 3}, .color = {255, 255, 0}},
};

constexpr psyqo::Vec3 c_cubePositions[CubeScene::NUM_CUBES] = {
    {.x = 0.0,  .y = 0.0,  .z = 0.0},
    {.x = 0.3,  .y = 0.0,  .z = 0.0},
    {.x = -0.3, .y = 0.0,  .z = 0.0},
    {.x = 0.0,  .y = 0.0,  .z = 0.3},
    {.x = 0.0,  .y = 0.0,  .z = -0.3},
};

// Floor extent on the XZ plane. PS1 +Y is screen-down, so the floor lives at +Y.
constexpr psyqo::FixedPoint<> c_roomMinX = -0.7_fp;
constexpr psyqo::FixedPoint<> c_roomMaxX =  0.7_fp;
constexpr psyqo::FixedPoint<> c_roomMinZ = -0.7_fp;
constexpr psyqo::FixedPoint<> c_roomMaxZ =  0.7_fp;
constexpr psyqo::FixedPoint<> c_roomFloorY =  0.4_fp;

constexpr Quad3D c_roomQuads[CubeScene::NUM_ROOM_QUADS] = {
    // Floor (camera looks down at it)
    {{{.x = c_roomMinX, .y = c_roomFloorY, .z = c_roomMinZ},
      {.x = c_roomMaxX, .y = c_roomFloorY, .z = c_roomMinZ},
      {.x = c_roomMinX, .y = c_roomFloorY, .z = c_roomMaxZ},
      {.x = c_roomMaxX, .y = c_roomFloorY, .z = c_roomMaxZ}},
     {.r = 60, .g = 60, .b = 60}},
};

constexpr psyqo::FixedPoint<> c_moveSpeed = 0.01_fp;

// Fixed pitch for the isometric-ish camera. ~30 degrees down (atan(0.7/1.0)
// roughly aims at the origin from the starting camera position).
constexpr psyqo::Angle c_cameraPitch = 0.17_pi;

} // namespace

void CubeScene::start(StartReason) {
    psyqo::GTE::clear<psyqo::GTE::Register::TRX, psyqo::GTE::Unsafe>();
    psyqo::GTE::clear<psyqo::GTE::Register::TRY, psyqo::GTE::Unsafe>();
    psyqo::GTE::clear<psyqo::GTE::Register::TRZ, psyqo::GTE::Unsafe>();

    psyqo::GTE::write<psyqo::GTE::Register::OFX, psyqo::GTE::Unsafe>(psyqo::FixedPoint<16>(160.0).raw());
    psyqo::GTE::write<psyqo::GTE::Register::OFY, psyqo::GTE::Unsafe>(psyqo::FixedPoint<16>(120.0).raw());
    psyqo::GTE::write<psyqo::GTE::Register::H,   psyqo::GTE::Unsafe>(120);
    // ZSF3 / ZSF4 are the depth-averaging scales the GTE uses to compute OTZ.
    // The classic heuristic ZSF3 = OT_SIZE / 3 ties render distance to the OT
    // size, so growing the OT alone doesn't push the far plane back. Keeping
    // ZSF3 smaller (relative to OT_SIZE) extends the visible depth range at
    // the cost of slightly coarser depth-sort resolution.
    psyqo::GTE::write<psyqo::GTE::Register::ZSF3, psyqo::GTE::Unsafe>(ORDERING_TABLE_SIZE / 12);
    psyqo::GTE::write<psyqo::GTE::Register::ZSF4, psyqo::GTE::Unsafe>(ORDERING_TABLE_SIZE / 16);
}

void CubeScene::frame() {
    auto &gpu = g_madnightEngine.gpu();
    auto &trig = g_madnightEngine.m_trig;
    auto &pad = g_madnightEngine.m_input;
    using Btn = psyqo::AdvancedPad::Button;
    constexpr auto P = psyqo::AdvancedPad::Pad::Pad1a;

    // ---- Input ----
    if (pad.isButtonPressed(P, Btn::L2)) m_cameraYaw -= 0.01_pi;
    if (pad.isButtonPressed(P, Btn::R2)) m_cameraYaw += 0.01_pi;

    psyqo::FixedPoint<> sinY = trig.sin(m_cameraYaw);
    psyqo::FixedPoint<> cosY = trig.cos(m_cameraYaw);
    psyqo::Vec3 forward = {.x = -sinY, .y = 0.0_fp, .z = cosY};
    psyqo::Vec3 right   = {.x = cosY,  .y = 0.0_fp, .z = sinY};

    auto applyMove = [&](const psyqo::Vec3 &dir, psyqo::FixedPoint<> amount) {
        m_cameraPos.x += dir.x * amount;
        m_cameraPos.y += dir.y * amount;
        m_cameraPos.z += dir.z * amount;
    };

    if (pad.isButtonPressed(P, Btn::Up))    applyMove(forward,  c_moveSpeed);
    if (pad.isButtonPressed(P, Btn::Down))  applyMove(forward, -c_moveSpeed);
    if (pad.isButtonPressed(P, Btn::Right)) applyMove(right,    c_moveSpeed);
    if (pad.isButtonPressed(P, Btn::Left))  applyMove(right,   -c_moveSpeed);

    // ---- Render setup ----
    int parity = gpu.getParity();
    auto &ot = m_ots[parity];
    auto &clear = m_clear[parity];
    auto &quads = m_quads[parity];

    static constexpr psyqo::Color c_bg = {.r = 0, .g = 0, .b = 0};
    gpu.getNextClear(clear.primitive, c_bg);
    gpu.chain(clear);

    // View rotation = inverse of (Ry(yaw) * Rx(pitch)) = Rx(-pitch) * Ry(-yaw).
    // Yaw turns the world around the camera, then pitch tilts it for the high
    // 3/4 angle.
    psyqo::Matrix33 yawRot =
        psyqo::SoftMath::generateRotationMatrix33(-m_cameraYaw, psyqo::SoftMath::Axis::Y, trig);
    psyqo::Matrix33 pitchRot =
        psyqo::SoftMath::generateRotationMatrix33(-c_cameraPitch, psyqo::SoftMath::Axis::X, trig);
    psyqo::Matrix33 viewRot;
    psyqo::SoftMath::multiplyMatrix33(pitchRot, yawRot, &viewRot);

    eastl::array<psyqo::Vertex, 4> projected;
    unsigned quadCount = 0;

    // Helper: project 4 vertices already in view space via the GTE and insert
    // them into the ordering table as a Quad. Returns whether the face was
    // emitted (false = backface culled or out of OT range).
    auto emitQuad = [&](const psyqo::Vec3 &a, const psyqo::Vec3 &b, const psyqo::Vec3 &c,
                        const psyqo::Vec3 &d, psyqo::Color color, bool cullBackface) -> bool {
        psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::V0>(a);
        psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::V1>(b);
        psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::V2>(c);

        psyqo::GTE::Kernels::rtpt();
        psyqo::GTE::Kernels::nclip();

        int32_t mac0 = 0;
        psyqo::GTE::read<psyqo::GTE::Register::MAC0>(reinterpret_cast<uint32_t *>(&mac0));
        if (cullBackface && mac0 <= 0) return false;

        psyqo::GTE::read<psyqo::GTE::Register::SXY0>(&projected[0].packed);
        psyqo::GTE::writeSafe<psyqo::GTE::PseudoRegister::V0>(d);
        psyqo::GTE::Kernels::rtps();

        psyqo::GTE::Kernels::avsz4();
        int32_t zIndex = 0;
        psyqo::GTE::read<psyqo::GTE::Register::OTZ>(reinterpret_cast<uint32_t *>(&zIndex));
        if (zIndex < 0 || zIndex >= int32_t(ORDERING_TABLE_SIZE)) return false;

        psyqo::GTE::read<psyqo::GTE::Register::SXY0>(&projected[1].packed);
        psyqo::GTE::read<psyqo::GTE::Register::SXY1>(&projected[2].packed);
        psyqo::GTE::read<psyqo::GTE::Register::SXY2>(&projected[3].packed);

        auto &quad = quads[quadCount++];
        quad.primitive.setPointA(projected[0]);
        quad.primitive.setPointB(projected[1]);
        quad.primitive.setPointC(projected[2]);
        quad.primitive.setPointD(projected[3]);
        quad.primitive.setColor(color);
        quad.primitive.setOpaque();

        ot.insert(quad, zIndex);
        return true;
    };

    // ---- Room (world-space geometry, no model rotation) ----
    {
        psyqo::Vec3 negCamPos = {.x = -m_cameraPos.x, .y = -m_cameraPos.y, .z = -m_cameraPos.z};
        psyqo::Vec3 viewTranslation;
        psyqo::SoftMath::matrixVecMul3(viewRot, negCamPos, &viewTranslation);
        psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Rotation>(viewRot);
        psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Translation>(viewTranslation);

        for (const auto &q : c_roomQuads) {
            emitQuad(q.v[0], q.v[1], q.v[2], q.v[3], q.color, /*cullBackface=*/false);
        }
    }

    // ---- Cubes (per-cube self rotation + view rotation) ----
    psyqo::Matrix33 selfRotX =
        psyqo::SoftMath::generateRotationMatrix33(m_selfRot, psyqo::SoftMath::Axis::X, trig);
    psyqo::Matrix33 selfRotY =
        psyqo::SoftMath::generateRotationMatrix33(m_selfRot, psyqo::SoftMath::Axis::Y, trig);
    psyqo::Matrix33 selfRot;
    psyqo::SoftMath::multiplyMatrix33(selfRotX, selfRotY, &selfRot);

    psyqo::Matrix33 gteRot;
    psyqo::SoftMath::multiplyMatrix33(viewRot, selfRot, &gteRot);
    psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Rotation>(gteRot);

    for (unsigned cubeIdx = 0; cubeIdx < NUM_CUBES; ++cubeIdx) {
        psyqo::Vec3 relative = {
            .x = c_cubePositions[cubeIdx].x - m_cameraPos.x,
            .y = c_cubePositions[cubeIdx].y - m_cameraPos.y,
            .z = c_cubePositions[cubeIdx].z - m_cameraPos.z,
        };
        psyqo::Vec3 viewTranslation;
        psyqo::SoftMath::matrixVecMul3(viewRot, relative, &viewTranslation);
        psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Translation>(viewTranslation);

        for (auto face : c_cubeFaces) {
            emitQuad(c_cubeVertices[face.vertices[0]], c_cubeVertices[face.vertices[1]],
                     c_cubeVertices[face.vertices[2]], c_cubeVertices[face.vertices[3]],
                     face.color, /*cullBackface=*/true);
        }
    }

    gpu.chain(ot);
    m_selfRot += 0.006_pi;
}
