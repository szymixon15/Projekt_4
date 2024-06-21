/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <algorithm>
#include <matplot/matplot.h>
#include <SFML/Audio.hpp> //biblioteka do dzwieku quadrotora
using namespace std;



Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 10, 10, 10, 1, 10, 0.25 / 2 / M_PI;
    R(0, 0) = 0.1;
    R(1, 1) = 0.1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}


void tworz_wykres_drogi(const std::vector<float>& wartosc_osi_x, const std::vector<float>& wartosc_osi_y) {
    using namespace matplot;
    auto fig = figure(true);
    plot(wartosc_osi_x, wartosc_osi_y);
    title("Historia lotu QUADROTORA");
    xlabel("OŚ X");
    ylabel("OŚ Y");
    show(); 

}

void ograniczenie_wylotu_quadrotora(Eigen::VectorXf& position, int screen_width, int screen_height) {

    if (position[0] < -screen_width / 2)
        position[0] = -screen_width / 2;
    else if (position[0] > screen_width / 2)
        position[0] = screen_width / 2;

    if (position[1] < -screen_height / 2)
        position[1] = -screen_height / 2;
    else if (position[1] > screen_height / 2)
        position[1] = screen_height / 2;
}


int main(int argc, char* args[]) {
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor, SCREEN_WIDTH, SCREEN_HEIGHT);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);

    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);

    vector<float> wartosc_osi_x;
    vector<float> wartosc_osi_y;

    // ładowanie pliku audio przy uzyciu biblioteki SFML //
    sf::SoundBuffer buffer;
    if (!buffer.loadFromFile("quadro_gora.wav")) {
        cerr << "Blad! Nie udalo sie wczytac pliku audio!" << std::endl;
        return -1;
    }

    // odtwarzanie pliku audio przy uzyciu biblioteki SFML //
    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.setLoop(true); // zapetlenie dzwieku //
    sound.play();

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0) {
        SDL_Event e;
        bool quit = false;

        while (!quit) {
            while (SDL_PollEvent(&e) != 0) {
                if (e.type == SDL_QUIT) {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                    int mysz_wsp_X, mysz_wsp_Y;
                    SDL_GetMouseState(&mysz_wsp_X, &mysz_wsp_Y);


                    // sprawdzanie czy quadrotor nie wylatuje poza naszą mape //
                    Eigen::VectorXf quadrotorPosition = Eigen::VectorXf::Zero(2);
                    ograniczenie_wylotu_quadrotora(quadrotorPosition, SCREEN_WIDTH, SCREEN_HEIGHT);

                    quadrotor_visualizer.przechwyc_wspolrzedne_myszki(mysz_wsp_X, mysz_wsp_Y);

                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p) {
                    sound.stop();
                    tworz_wykres_drogi(wartosc_osi_x, wartosc_osi_y);
                }
            }

            SDL_Delay(static_cast<int>(dt * 1000));

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            control(quadrotor, K);
            quadrotor.Update(dt);

  
            Eigen::VectorXf state = quadrotor.GetState(); // Pobranie aktualnego stanu quadrotora
            float predkosc_quadrotora = state.segment<3>(3).norm(); // wektor o długości 3, zaczynając od indeksu 3
            // (predkosc x, predkosc y i predkosc katowa) state'a + obliczanie euklidesowej normy //
            if (predkosc_quadrotora < 1.0f) {
                sound.setPitch(0.75f); // ustawienie czestotliwosci na 0.75 //
            }
            else if (predkosc_quadrotora < 1.25f) {
                sound.setPitch(0.9f); // ustawienie czestotliwosci na 0.9 //
            }
            else if (predkosc_quadrotora < 1.5f) {
                sound.setPitch(1.05f); // ustawienie czestotliwosci na 1.05 //
            }
            else if (predkosc_quadrotora < 1.75f) {
                sound.setPitch(1.2f); // ustawienie czestotliwosci na 1.2 //
            }
            else if (predkosc_quadrotora < 2.0f) {
                sound.setPitch(1.35f); // ustawienie czestotliwosci na 1.35 //
            }
            else {
                sound.setPitch(1.5f); // ustawienie czestotliwosci na 1.5 //
            }


            wartosc_osi_x.push_back(quadrotor.GetState()[0]);
            wartosc_osi_y.push_back(quadrotor.GetState()[1]);
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
