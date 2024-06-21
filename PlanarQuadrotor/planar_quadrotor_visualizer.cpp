#include "planar_quadrotor_visualizer.h"
#include <algorithm>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>
#include <cmath>

const int MAX_SCREEN_WIDTH = 1280;
const int MAX_SCREEN_HEIGHT = 720;

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr, int screen_width, int screen_height)
    : quadrotor_ptr(quadrotor_ptr), screen_width(screen_width), screen_height(screen_height) {}


void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float wartosc_x = state[0] * (screen_width / 2) / 1.0; // Skalowanie do metra wg poelcenia
    float wartosc_y = state[1] * (screen_height / 2) / 1.0; // Skalowanie do metra wg poelcenia
    float kat_theta = -state[2]; // potrzebny do przechy³u quadrotora


    int srodek_ekranu_X = screen_width / 2;
    int srodek_ekranu_Y = screen_height / 2;
    int quadrotor_wspolrzedna_X = static_cast<int>(srodek_ekranu_X + wartosc_x);
    int quadrotor_wspolrzedna_Y = static_cast<int>(srodek_ekranu_Y - wartosc_y);

    // wymiary czlonu quadrotoru
    int szerokosc_czlonu = 80;
    int wysokosc_czlonu = 10;


    float polowa_szerokosc_czlonu = szerokosc_czlonu / 2.0f;
    float polowa_wysokosc_czlonu = wysokosc_czlonu / 2.0f;

    float cos_theta = cos(kat_theta);
    float sin_theta = sin(kat_theta);

  
    Sint16 glowny_czlon_quadrotoru_wsp_x[4];
    Sint16 glowny_czlon_quadrotoru_wsp_y[4];

    glowny_czlon_quadrotoru_wsp_x[0] = quadrotor_wspolrzedna_X + (-polowa_szerokosc_czlonu * cos_theta - (-polowa_wysokosc_czlonu) * sin_theta);
    glowny_czlon_quadrotoru_wsp_y[0] = quadrotor_wspolrzedna_Y + (-polowa_szerokosc_czlonu * sin_theta + (-polowa_wysokosc_czlonu) * cos_theta);

    glowny_czlon_quadrotoru_wsp_x[1] = quadrotor_wspolrzedna_X + (polowa_szerokosc_czlonu * cos_theta - (-polowa_wysokosc_czlonu) * sin_theta);
    glowny_czlon_quadrotoru_wsp_y[1] = quadrotor_wspolrzedna_Y + (polowa_szerokosc_czlonu * sin_theta + (-polowa_wysokosc_czlonu) * cos_theta);

    glowny_czlon_quadrotoru_wsp_x[2] = quadrotor_wspolrzedna_X + (polowa_szerokosc_czlonu * cos_theta - (polowa_wysokosc_czlonu)*sin_theta);
    glowny_czlon_quadrotoru_wsp_y[2] = quadrotor_wspolrzedna_Y + (polowa_szerokosc_czlonu * sin_theta + (polowa_wysokosc_czlonu)*cos_theta);

    glowny_czlon_quadrotoru_wsp_x[3] = quadrotor_wspolrzedna_X + (-polowa_szerokosc_czlonu * cos_theta - (polowa_wysokosc_czlonu)*sin_theta);
    glowny_czlon_quadrotoru_wsp_y[3] = quadrotor_wspolrzedna_Y + (-polowa_szerokosc_czlonu * sin_theta + (polowa_wysokosc_czlonu)*cos_theta);

    // kolor czlonu quadrotoru
    SDL_SetRenderDrawColor(gRenderer.get(), 0x70, 0x70, 0x70, 0xFF);

    // rysowanie czlonu quadrotoru
    filledPolygonRGBA(gRenderer.get(), glowny_czlon_quadrotoru_wsp_x, glowny_czlon_quadrotoru_wsp_y, 4, 0x70, 0x70, 0x70, 0xFF);

    // wymiary ramion
    int szerokosc_ramienia = 5;
    int wysokosc_ramienia = 30;


    Sint16 lewe_ramie_wspolrzedne_x[4];
    Sint16 lewe_ramie_wspolrzedne_y[4];

    float lewe_ramie_srodek_x = quadrotor_wspolrzedna_X - polowa_szerokosc_czlonu * cos_theta;
    float lewe_ramie_srodek_y = quadrotor_wspolrzedna_Y - polowa_szerokosc_czlonu * sin_theta - 10;

    lewe_ramie_wspolrzedne_x[0] = lewe_ramie_srodek_x + (-szerokosc_ramienia / 2.0f * cos_theta - (-wysokosc_ramienia / 2.0f) * sin_theta);
    lewe_ramie_wspolrzedne_y[0] = lewe_ramie_srodek_y + (-szerokosc_ramienia / 2.0f * sin_theta + (-wysokosc_ramienia / 2.0f) * cos_theta);

    lewe_ramie_wspolrzedne_x[1] = lewe_ramie_srodek_x + (szerokosc_ramienia / 2.0f * cos_theta - (-wysokosc_ramienia / 2.0f) * sin_theta);
    lewe_ramie_wspolrzedne_y[1] = lewe_ramie_srodek_y + (szerokosc_ramienia / 2.0f * sin_theta + (-wysokosc_ramienia / 2.0f) * cos_theta);

    lewe_ramie_wspolrzedne_x[2] = lewe_ramie_srodek_x + (szerokosc_ramienia / 2.0f * cos_theta - (wysokosc_ramienia / 2.0f) * sin_theta);
    lewe_ramie_wspolrzedne_y[2] = lewe_ramie_srodek_y + (szerokosc_ramienia / 2.0f * sin_theta + (wysokosc_ramienia / 2.0f) * cos_theta);

    lewe_ramie_wspolrzedne_x[3] = lewe_ramie_srodek_x + (-szerokosc_ramienia / 2.0f * cos_theta - (wysokosc_ramienia / 2.0f) * sin_theta);
    lewe_ramie_wspolrzedne_y[3] = lewe_ramie_srodek_y + (-szerokosc_ramienia / 2.0f * sin_theta + (wysokosc_ramienia / 2.0f) * cos_theta);

   
    Sint16 prawe_ramie_wspolrzedne_x[4];
    Sint16 prawe_ramie_wspolrzedne_y[4];

    float prawe_ramie_srodek_x = quadrotor_wspolrzedna_X + polowa_szerokosc_czlonu * cos_theta;
    float prawe_ramie_srodek_y = quadrotor_wspolrzedna_Y + polowa_szerokosc_czlonu * sin_theta - 10;

    prawe_ramie_wspolrzedne_x[0] = prawe_ramie_srodek_x + (-szerokosc_ramienia / 2.0f * cos_theta - (-wysokosc_ramienia / 2.0f) * sin_theta);
    prawe_ramie_wspolrzedne_y[0] = prawe_ramie_srodek_y + (-szerokosc_ramienia / 2.0f * sin_theta + (-wysokosc_ramienia / 2.0f) * cos_theta);

    prawe_ramie_wspolrzedne_x[1] = prawe_ramie_srodek_x + (szerokosc_ramienia / 2.0f * cos_theta - (-wysokosc_ramienia / 2.0f) * sin_theta);
    prawe_ramie_wspolrzedne_y[1] = prawe_ramie_srodek_y + (szerokosc_ramienia / 2.0f * sin_theta + (-wysokosc_ramienia / 2.0f) * cos_theta);

    prawe_ramie_wspolrzedne_x[2] = prawe_ramie_srodek_x + (szerokosc_ramienia / 2.0f * cos_theta - (wysokosc_ramienia / 2.0f) * sin_theta);
    prawe_ramie_wspolrzedne_y[2] = prawe_ramie_srodek_y + (szerokosc_ramienia / 2.0f * sin_theta + (wysokosc_ramienia / 2.0f) * cos_theta);

    prawe_ramie_wspolrzedne_x[3] = prawe_ramie_srodek_x + (-szerokosc_ramienia / 2.0f * cos_theta - (wysokosc_ramienia / 2.0f) * sin_theta);
    prawe_ramie_wspolrzedne_y[3] = prawe_ramie_srodek_y + (-szerokosc_ramienia / 2.0f * sin_theta + (wysokosc_ramienia / 2.0f) * cos_theta);

    // kolor ramion
    SDL_SetRenderDrawColor(gRenderer.get(), 0x3d, 0x3d, 0x3d, 0xFF);

    // rysowanie lewe ramie
    filledPolygonRGBA(gRenderer.get(), lewe_ramie_wspolrzedne_x, lewe_ramie_wspolrzedne_y, 4, 0x3d, 0x3d, 0x3d, 0xFF);

    // rysowanie prawe ramie
    filledPolygonRGBA(gRenderer.get(), prawe_ramie_wspolrzedne_x, prawe_ramie_wspolrzedne_y, 4, 0x3d, 0x3d, 0x3d, 0xFF);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x80, 0x04, 0xFF);

    // kolor smigiel
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x80, 0xFF, 0xFF);

    // wymiary smigiel
    float smiglo_dlugosc = 5.0f;

    float smiglo_srodek_X = quadrotor_wspolrzedna_X;
    float smiglo_srodek_Y = quadrotor_wspolrzedna_Y - 30;

    float smiglo_wsp_x1 = smiglo_srodek_X - (szerokosc_czlonu/2) * cos(kat_theta); 
    float smiglo_wsp_y1 = smiglo_srodek_Y - (szerokosc_czlonu / 2) * sin(kat_theta);

    float smiglo_wsp_x2 = smiglo_srodek_X + (szerokosc_czlonu / 2) * cos(kat_theta); 
    float smiglo_wsp_y2 = smiglo_srodek_Y + (szerokosc_czlonu / 2) * sin(kat_theta); 

    // rysowanie smigiel
    filledEllipseRGBA(gRenderer.get(), smiglo_wsp_x1, smiglo_wsp_y1, szerokosc_smigla_zmienna, smiglo_dlugosc, 0x00, 0x80, 0xFF, 0xFF);
    filledEllipseRGBA(gRenderer.get(), smiglo_wsp_x2, smiglo_wsp_y2, szerokosc_smigla_zmienna, smiglo_dlugosc, 0x00, 0x80, 0xFF, 0xFF);

 
    szerokosc_smigla_zmienna += 0.4f; 


    if (szerokosc_smigla_zmienna >= 40.0f) {
        szerokosc_smigla_zmienna = 10.0f; // Resetowanie o poczatku
    }

}

void PlanarQuadrotorVisualizer::przechwyc_wspolrzedne_myszki(int x, int y) {

    float wartosc_x = static_cast<float>(x - screen_width / 2) / (screen_width / 2) * 1.0;
    float wartosc_y = static_cast<float>(screen_height / 2 - y) / (screen_height / 2) * 1.0;

    // ograniczenie wylotu //

    if (wartosc_x < -1.0)
        wartosc_x = -1.0;
    else if (wartosc_x > 1.0)
        wartosc_x = 1.0;

    if (wartosc_y < -1.0)
        wartosc_y = -1.0;
    else if (wartosc_y > 1.0)
        wartosc_y = 1.0;

    // nowe wspolrzedne - cel
    Eigen::VectorXf new_goal = Eigen::VectorXf::Zero(6);
    new_goal[0] = wartosc_x;
    new_goal[1] = wartosc_y;
    new_goal[2] = 0; 
    quadrotor_ptr->SetGoal(new_goal);
}
