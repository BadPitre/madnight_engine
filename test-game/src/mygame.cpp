#include "mygame.hh"
#include "cube_scene.hh"
#include "madnight.hh"
#include "game.hh"
#include "psyqo/xprintf.h"

MadnightGame g_myGame;
MadnightEngineGame &g_madnightEngineGame = g_myGame;
static CubeScene cubeScene;

psyqo::Coroutine<> MadnightGame::InitialLoad(void)
{
    printf("welcome to your game code!\n");
    g_madnightEngine.SwitchScene(&cubeScene);
    co_return;
}

int main() { return g_madnightEngine.run(); }
