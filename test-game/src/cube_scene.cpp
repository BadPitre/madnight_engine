#include "cube_scene.hh"
#include "madnight.hh"

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

// Camera orbits at this radius around the world origin, on the XZ plane.
constexpr psyqo::FixedPoint<> c_cameraRadius = 0.9_fp;

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

    int parity = gpu.getParity();
    auto &ot = m_ots[parity];
    auto &clear = m_clear[parity];
    auto &quads = m_quads[parity];

    static constexpr psyqo::Color c_bg = {.r = 20, .g = 20, .b = 40};
    gpu.getNextClear(clear.primitive, c_bg);
    gpu.chain(clear);

    // Camera world position: orbits around origin in the XZ plane at fixed radius.
    // psyqo's Y rotation Ry(a) maps (0,0,-1) to (sin(a), 0, -cos(a)), so the camera
    // world rotation by m_cameraAngle puts it at this position when the "base" is (0,0,-R).
    psyqo::FixedPoint<> sinA = trig.sin(m_cameraAngle);
    psyqo::FixedPoint<> cosA = trig.cos(m_cameraAngle);
    psyqo::Vec3 cameraPos = {
        .x = c_cameraRadius * sinA,
        .y = 0.0_fp,
        .z = -c_cameraRadius * cosA,
    };

    // View rotation = inverse of camera world rotation = Ry(-m_cameraAngle).
    psyqo::Matrix33 viewRot =
        psyqo::SoftMath::generateRotationMatrix33(-m_cameraAngle, psyqo::SoftMath::Axis::Y, trig);

    // Each cube's self rotation: combine X and Y so it spins on two axes.
    psyqo::Matrix33 selfRotX =
        psyqo::SoftMath::generateRotationMatrix33(m_selfRot, psyqo::SoftMath::Axis::X, trig);
    psyqo::Matrix33 selfRotY =
        psyqo::SoftMath::generateRotationMatrix33(m_selfRot, psyqo::SoftMath::Axis::Y, trig);
    psyqo::Matrix33 selfRot;
    psyqo::SoftMath::multiplyMatrix33(selfRotX, selfRotY, &selfRot);

    // Combined rotation written to GTE Rotation: viewRot * selfRot.
    psyqo::Matrix33 gteRot;
    psyqo::SoftMath::multiplyMatrix33(viewRot, selfRot, &gteRot);
    psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Rotation>(gteRot);

    eastl::array<psyqo::Vertex, 4> projected;
    unsigned quadCount = 0;

    for (unsigned cubeIdx = 0; cubeIdx < NUM_CUBES; ++cubeIdx) {
        // View-space translation = viewRot * (cube_world_pos - camera_pos).
        psyqo::Vec3 relative = {
            .x = c_cubePositions[cubeIdx].x - cameraPos.x,
            .y = c_cubePositions[cubeIdx].y - cameraPos.y,
            .z = c_cubePositions[cubeIdx].z - cameraPos.z,
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

    m_cameraAngle += 0.003_pi;
    m_selfRot += 0.006_pi;
}
