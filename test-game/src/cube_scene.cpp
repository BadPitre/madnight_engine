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
    int parity = gpu.getParity();

    auto &ot = m_ots[parity];
    auto &clear = m_clear[parity];

    static constexpr psyqo::Color c_bg = {.r = 20, .g = 20, .b = 40};
    gpu.getNextClear(clear.primitive, c_bg);
    gpu.chain(clear);

    psyqo::GTE::write<psyqo::GTE::Register::TRZ, psyqo::GTE::Unsafe>(900);

    auto transform = psyqo::SoftMath::generateRotationMatrix33(m_rot, psyqo::SoftMath::Axis::X, g_madnightEngine.m_trig);
    auto roty = psyqo::SoftMath::generateRotationMatrix33(m_rot, psyqo::SoftMath::Axis::Y, g_madnightEngine.m_trig);
    psyqo::SoftMath::multiplyMatrix33(transform, roty, &transform);
    psyqo::GTE::writeUnsafe<psyqo::GTE::PseudoRegister::Rotation>(transform);

    eastl::array<psyqo::Vertex, 4> projected;
    int faceNum = 0;

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

        auto &quad = m_quads[faceNum];
        quad.primitive.setPointA(projected[0]);
        quad.primitive.setPointB(projected[1]);
        quad.primitive.setPointC(projected[2]);
        quad.primitive.setPointD(projected[3]);
        quad.primitive.setColor(face.color);
        quad.primitive.setOpaque();

        ot.insert(quad, zIndex);
        faceNum++;
    }

    gpu.chain(ot);
    m_rot += 0.005_pi;
}
