#ifndef _CUBE_SCENE_H
#define _CUBE_SCENE_H

#include "psyqo/fragments.hh"
#include "psyqo/ordering-table.hh"
#include "psyqo/primitives/common.hh"
#include "psyqo/primitives/quads.hh"
#include "psyqo/scene.hh"
#include "psyqo/trigonometry.hh"
#include "psyqo/vector.hh"

class CubeScene final : public psyqo::Scene {
  public:
    static constexpr unsigned ORDERING_TABLE_SIZE = 1024;
    static constexpr unsigned NUM_CUBE_FACES = 6;
    static constexpr unsigned NUM_CUBES = 5;
    static constexpr unsigned MAX_QUADS = NUM_CUBES * NUM_CUBE_FACES;

  private:
    void start(StartReason reason) override;
    void frame() override;

    psyqo::Angle m_cameraAngle = 0;
    psyqo::Angle m_selfRot = 0;

    psyqo::OrderingTable<ORDERING_TABLE_SIZE> m_ots[2];
    psyqo::Fragments::SimpleFragment<psyqo::Prim::FastFill> m_clear[2];
    eastl::array<psyqo::Fragments::SimpleFragment<psyqo::Prim::Quad>, MAX_QUADS> m_quads[2];
};

#endif
