#include "application.h"
#include <imgui.h>

#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>

#include "../boids/boids.h"


class TestApp : public Application
{
#define COLOR_OUT    nvgRGBA(220,50,50,255)
#define COLOR_IN     nvgRGBA(50,50,220,255)
#define COLOR_SOLVED nvgRGBA(50,220,50,255)


public:

    TestApp(int w, int h, const char * title) : Application(title, w, h) {
        
        ImGui::StyleColorsClassic();
        
        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);
        
    }

    void process() override {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./60. * 1.e6)
        {
            if(keyDown[GLFW_KEY_R])
                boids.initializePositions(currentMethod);
            if(keyDown[GLFW_KEY_SPACE])
                boids.pause();
            if(keyDown[GLFW_KEY_ESCAPE])
                exit(0);
            lastFrame = now;
        }
    }

    void drawImGui() override {

        using namespace ImGui;

        const char* names[] = {"FreeFall", "Separation", "Alignment", "Cohesion", "Leading", "Circular", "Collision Avoidance", "Predator Pray"};
        const char* integrations[] = {"Explicit Euler", "Sympletic Euler", "Explicit Midpoint"};
       
        Begin("Menu");
        Combo("Boids Behavior", (int*)&currentMethod, names, 8);
        Combo("Integration Method", (int*)&currentIntMet, integrations, 3);
        InputDouble("Step size", &step_size, 0.0001, 10);
        End();
    }

    void drawNanoVG() override {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        TV mouse_pos_screen;
        VectorXT mouse_pos_norm(2, 1);
        T scale = 0.4;

        auto shift_screen_to_01 = [](TV pos_screen, T scale, T width, T height)
        {
            return TV((pos_screen[0]/1.5 - 0.5 * (0.5 - scale) * width) / (scale  * width), (pos_screen[1]/1.45 - 0.5 * (0.5 - scale) * height) / (scale * height));
        };

        mouse_pos_screen << mouse_pos.x, mouse_pos.y;

        //std::cout << "Mouse \n" << mouse_state.lastMouseX << "\n\n";

        TV mouse_01 = shift_screen_to_01(TV(mouse_pos_screen[0], mouse_pos_screen[1]), scale, width, height);

        mouse_pos_norm << mouse_01[0], mouse_01[1];


        boids.updateBehavior(currentMethod, currentIntMet, step_size, mouse_pos_norm);
        
        VectorXT boids_pos = boids.getPositions();
        VectorXT boids_group = boids.getGroup();

        auto shift_01_to_screen = [](TV pos_01, T scale, T width, T height)
        {
            return TV(0.5 * (0.5 - scale) * width + scale * pos_01[0] * width, 0.5 * (0.5 - scale) * height + scale * pos_01[1] * height);
        };

        if (currentMethod==4){
            TV pos = boids_pos.segment<2>(0);
            nvgBeginPath(vg);
    
            T scale = 0.4;
            TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
            nvgCircle(vg, screen_pos[0], screen_pos[1], 4.f);
            nvgFillColor(vg, nvgRGBA(50,50,220,255));
            nvgFill(vg);

            for(int i = 1; i < boids_pos.size()/2; i++){
                pos = boids_pos.segment<2>(i * 2);
                nvgBeginPath(vg);

                screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, COLOR_OUT);
                nvgFill(vg);
            }
        }
        
        else if (currentMethod==7){
            for(int i = 0; i < boids_pos.size()/2; i++){
                TV pos = boids_pos.segment<2>(i * 2);
                nvgBeginPath(vg);
        
                // just map position from 01 simulation space to scree space
                // feel free to make changes
                // the only thing that matters is you have pos computed correctly from your simulation
                T scale = 0.4;
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                if (boids_group(i)==0){
                    nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                    nvgFillColor(vg, COLOR_OUT);
                    nvgFill(vg);
                }
                else if (boids_group(i)==1){
                    nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                    nvgFillColor(vg, nvgRGBA(50,50,220,255));
                    nvgFill(vg);
                }
            }
        }

        else{
            for(int i = 0; i < boids_pos.size()/2; i++){
                TV pos = boids_pos.segment<2>(i * 2);
                nvgBeginPath(vg);
        
                // just map position from 01 simulation space to scree space
                // feel free to make changes
                // the only thing that matters is you have pos computed correctly from your simulation
                T scale = 0.4;
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, COLOR_OUT);
                nvgFill(vg);
            }
        }

        if (currentMethod==6 || currentMethod==4){
            TV center;
            center << 0.7, 0.7;
            double ray = 0.2;

            nvgBeginPath(vg);
    
            T scale = 0.4;
            TV center_pos = shift_01_to_screen(TV(center[0], center[1]), scale, width, height);
            TV ray_shift = shift_01_to_screen(TV(center[0] - ray, center[1] - ray), scale, width, height);

            nvgCircle(vg, center_pos[0], center_pos[1], center_pos[0] - ray_shift[0]);
            nvgFillColor(vg, nvgRGBA(100,200,50,255));
            nvgFill(vg);
        }
    }


protected:
    void mouseButtonPressed(int button, int mods) override {

    }

    void mouseButtonReleased(int button, int mods) override {
        
    }

private:
    int loadFonts(NVGcontext* vg)
    {
        int font;
        font = nvgCreateFont(vg, "sans", "../example/Roboto-Regular.ttf");
        if (font == -1) {
            printf("Could not add font regular.\n");
            return -1;
        }
        font = nvgCreateFont(vg, "sans-bold", "../example/Roboto-Bold.ttf");
        if (font == -1) {
            printf("Could not add font bold.\n");
            return -1;
        }
        return 0;
    }

private:

    MethodTypes currentMethod = FREEFALL;
    IntegrationMethods currentIntMet = EXPLICIT;
    double step_size = 0.01;

    MouseState mouse_state;


    Boids boids = Boids(40, currentMethod);
    std::chrono::high_resolution_clock::time_point lastFrame;
};

int main(int, char**)
{
    int width = 720;
    int height = 720;
    std::cout << " main " << std::endl;
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();

    return 0;
}
