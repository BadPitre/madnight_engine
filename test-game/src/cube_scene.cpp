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

constexpr psyqo::FixedPoint<> c_moveSpeed = 0.01_fp;

} // namespace

void CubeScene::start(StartReason) {
    psyqo::GTE::clear<psyqo::GTE::Register::TRX, psyqo::GTE::Unsafe>();
    psyqo::GTE::clear<psyqo::GTE::Register::TRY, psyqo::GTE::Unsafe>();
    psyqo::GTE::clear<psyqo::GTE::Register::TRZ, psyqo::GTE::Unsafe>();

    psyqo::GTE::write<psyqo::GTE::Register::OFX, psyqo::GTE::Unsafe>(psyqo::FixedPoint<16>(160.0).raw());
    psyqo::GTE::write<psyqo::GTE::Register::OFY, psyqo::GTE::Unsafe>(psyqo::FixedPoint<16>(120.0).raw());
    psyqo::GTE::write<psyqo::GTE::Register::H,   psyqo::GTE::Unsafe>(120);
    psyqo::GTE::write<psyqo::GTE::Register::ZSF3, psyqo::GTE::Unsafe>(ORDERING_TABLE_SIZE / 3);
    psyqo::GTE::write<psyqo::GTE::Register::ZSF4, psyqo::GTE::Unsafe>(ORDERING_TABLE_SIZE / 4);
}

void CubeScene::frame() {
    auto &gpu = g_madnightEngine.gpu();
    auto &trig = g_madnightEngine.m_trig;
    auto &pad = g_madnightEngine.m_input;
    using Btn = psyqo::AdvancedPad::Button;
    constexpr auto P = psyqo::AdvancedPad::Pad::Pad1a;

    // ---- Input: rotate camera with L2/R2 ----
    if (pad.isButtonPressed(P, Btn::L2)) m_cameraYaw -= 0.01_pi;
    if (pad.isButtonPressed(P, Btn::R2)) m_cameraYaw += 0.01_pi;

    // ---- Input: move camera in its local XZ plane with the D-pad ----
    // psyqo Y rotation Ry(yaw) maps (0,0,1) -> (-sin(yaw), 0, cos(yaw))
    // and (1,0,0) -> (cos(yaw), 0, sin(yaw)).
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

    // ---- Render ----
    int parity = gpu.getParity();
    auto &ot = m_ots[parity];
    auto &clear = m_clear[parity];
    auto &quads = m_quads[parity];

    static constexpr psyqo::Color c_bg = {.r = 20, .g = 20, .b = 40};
    gpu.getNextClear(clear.primitive, c_bg);
    gpu.chain(clear);

    // View rotation = inverse of camera world yaw = Ry(-yaw).
    psyqo::Matrix33 viewRot =
        psyqo::SoftMath::generateRotationMatrix33(-m_cameraYaw, psyqo::SoftMath::Axis::Y, trig);

    // Self-rotation for each cube: X + Y combined.
    psyqo::Matrix33 selfRotX =
        psyqo::SoftMath::generateRotationMatrix33(m_selfRot, psyqo::SoftMath::Axis::X, trig);
    psyqo::Matrix33 selfRotY =
        psyqo::SoftMath::generateRotationMatrix33(m_selfRot, psyqo::SoftMath::Axis::Y, trig);
    psyqo::Matrix33 selfRot;
    psyqo::SoftMath::multiplyMatrix33(selfRotX, selfRotY, &selfRot);

    psyqo::Matrix33 gteRot;
    psyqo::SoftMath::multiplyMatrix33(viewRot, selfRot, &gteRot);
    psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Rotation>(gteRot);

    eastl::array<psyqo::Vertex, 4> projected;
    unsigned quadCount = 0;

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
            psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::V0>(c_cubeVertices[face.vertices[0]]);
            psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::V1>(c_cubeVertices[face.vertices[1]]);
            psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::V2>(c_cubeVertices[face.vertices[2]]);

            psyqo::GTE::Kernels::rtpt();
            psyqo::GTE::Kernels::nclip();

            int32_t mac0 = 0;
            psyqo::GTE::read<psyqo::GTE::Register::MAC0>(reinterpret_cast<uint32_t *>(&mac0));
            if (mac0 <= 0) continue;

            psyqo::GTE::read<psyqo::GTE::Register::SXY0>(&projected[0].packed);
            psyqo::GTE::writeSafe<psyqo::GTE::PseudoRegister::V0>(c_cubeVertices[face.vertices[3]]);
            psyqo::GTE::Kernels::rtps();

            psyqo::GTE::Kernels::avsz4();
            int32_t zIndex = 0;
            psyqo::GTE::read<psyqo::GTE::Register::OTZ>(reinterpret_cast<uint32_t *>(&zIndex));
            if (zIndex < 0 || zIndex >= int32_t(ORDERING_TABLE_SIZE)) continue;

            psyqo::GTE::read<psyqo::GTE::Register::SXY0>(&projected[1].packed);
            psyqo::GTE::read<psyqo::GTE::Register::SXY1>(&projected[2].packed);
            psyqo::GTE::read<psyqo::GTE::Register::SXY2>(&projected[3].packed);

            auto &quad = quads[quadCount++];
            quad.primitive.setPointA(projected[0]);
            quad.primitive.setPointB(projected[1]);
            quad.primitive.setPointC(projected[2]);
            quad.primitive.setPointD(projected[3]);
            quad.primitive.setColor(face.color);
            quad.primitive.setOpaque();

            ot.insert(quad, zIndex);
        }
    }

    gpu.chain(ot);

    m_selfRot += 0.006_pi;
}
