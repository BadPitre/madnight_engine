#ifndef _CUBE_SCENE_H
#define _CUBE_SCENE_H

#include "psyqo/fragments.hh"
#include "psyqo/ordering-table.hh"
#include "psyqo/primitives/common.hh"
#include "psyqo/primitives/misc.hh"
#include "psyqo/primitives/quads.hh"
#include "psyqo/scene.hh"
#include "psyqo/trigonometry.hh"
#include "psyqo/vector.hh"

class CubeScene final : public psyqo::Scene {
    static constexpr unsigned ORDERING_TABLE_SIZE = 240;
    static constexpr unsigned NUM_CUBE_FACES = 6;

    void start(StartReason reason) override;
    void frame() override;

    psyqo::Angle m_rot = 0;
    psyqo::OrderingTable<ORDERING_TABLE_SIZE> m_ots[2];
    psyqo::Fragments::SimpleFragment<psyqo::Prim::FastFill> m_clear[2];
    eastl::array<psyqo::Fragments::SimpleFragment<psyqo::Prim::Quad>, NUM_CUBE_FACES> m_quads;
};

#endif
