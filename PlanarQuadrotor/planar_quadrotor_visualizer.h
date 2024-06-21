#pragma once

#include <memory>
#include <SDL.h>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>
#include "planar_quadrotor.h"
#include <algorithm>

class PlanarQuadrotorVisualizer {
private:
    PlanarQuadrotor* quadrotor_ptr;
    int screen_width;
    int screen_height;
    float szerokosc_smigla_zmienna = 10.0f;


public:
    PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr, int screen_width, int screen_height);
    void render(std::shared_ptr<SDL_Renderer>& gRenderer);
    void przechwyc_wspolrzedne_myszki(int x, int y);
};


