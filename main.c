#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
const SDL_Color RED = {255,0,0,255};
const SDL_Color GREEN = {0,255,0,255};
const SDL_Color BLUE = {0,0,255,255};
const SDL_Color BLACK = {0,0,0,255};
const SDL_Color WHITE = {255,255,255,255};

// Structure to represent a 2D vector
typedef struct {
    double x;
    double y;
} vector;

typedef struct {
    double x;
    double y;
    double radius;
} circle;

// Function to calculate the dot product of two vectors
double dot_product(vector v1, vector v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

// Function to calculate the magnitude (length) of a vector
double magnitude(vector v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

// Function to subtract two vectors
vector subtract_vectors(vector v1, vector v2) {
    vector result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    return result;
}

// Function to add two vectors
vector add_vectors(vector v1, vector v2) {
    vector result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    return result;
}

// Function to scale a vector by a scalar
vector scale_vector(vector v, double scalar) {
    vector result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    return result;
}

typedef struct {
    SDL_Rect rect;
    float Xspeed, Yspeed;
    float x,y;
    float mass;
    SDL_Color color;

}body;

// Function to calculate final velocities for a perfectly elastic 2D collision
int calculate_vector_collision(body *body1, body *body2) {
    double mass1 = body1->mass;
    vector starting_vector_1;
    starting_vector_1.x = body1->Xspeed;
    starting_vector_1.y = body1->Yspeed;
    double mass2 = body2->mass;
    vector starting_vector_2;
    starting_vector_2.x = body2->Xspeed;
    starting_vector_2.y = body2->Yspeed;

    // Check for zero masses
    if (mass1 <= 0.0 || mass2 <= 0.0) {
        printf("Error: Masses must be greater than zero.\n");
        return -1;
    }

    // Calculate the relative velocity
   //vctor v_rel = subtract_vectors(starting_vector_1, starting_vector_2);
    vector normal = subtract_vectors(starting_vector_1, starting_vector_2);

    // Calculate the normal vector (line of impact)
   /*
    vector normal;
    normal.x = v_rel.x;
    normal.y = v_rel.y;
    */
    // Normalize the normal vector.
    double norm_magnitude = magnitude(normal);

    if(norm_magnitude == 0){
        printf("Error: Relative velocity magnitude is zero.\n");
        return -1;
    }

    normal.x /= norm_magnitude;
    normal.y /= norm_magnitude;

    // Calculate the projection of velocities onto the normal vector
    double starting_vector_1_normal = dot_product(starting_vector_1, normal);
    double starting_vector_2_normal = dot_product(starting_vector_2, normal);

    // Calculate the final velocities along the normal
    double final_vector_1_normal = ((mass1 - mass2) * starting_vector_1_normal + 2 * mass2 * starting_vector_2_normal) / (mass1 + mass2);
    double final_vector_2_normal = (2 * mass1 * starting_vector_1_normal + (mass2 - mass1) * starting_vector_2_normal) / (mass1 + mass2);

    // Calculate the tangential vectors (perpendicular to normal)
    vector tangent;
    tangent.x = -normal.y;
    tangent.y = normal.x;

    // Project initial velocities onto the tangential
    double starting_vector_1_tangent = dot_product(starting_vector_1, tangent);
    double starting_vector_2_tangent = dot_product(starting_vector_2, tangent);

    // Calculate the final velocity vectors.
    vector final_vector_1_normal_vector = scale_vector(normal,final_vector_1_normal);
    vector final_vector_2_normal_vector = scale_vector(normal,final_vector_2_normal);
    vector final_vector_1_tangent_vector = scale_vector(tangent,starting_vector_1_tangent);
    vector final_vector_2_tangent_vector = scale_vector(tangent,starting_vector_2_tangent);


    vector final_velocity_1 = add_vectors(final_vector_1_normal_vector, final_vector_1_tangent_vector);
    vector final_velocity_2 = add_vectors(final_vector_2_normal_vector, final_vector_2_tangent_vector);

    body1->Xspeed = final_velocity_1.x;
    body1->Yspeed = final_velocity_1.y;
    body2->Xspeed = final_velocity_2.x;
    body2->Yspeed = final_velocity_2.y;

    return 0; // Indicate success
}

void set_color(SDL_Renderer *renderer, SDL_Color color){
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    return;
}


void draw_circle_octants(SDL_Renderer* renderer, circle circle, SDL_Color color) {
    set_color(renderer, color);
    int x = 0;
    int y = circle.radius;
    int d = 1 - circle.radius;
    int prev_x[8];
    int prev_y[8];

    int err = 0;
     //Initialize prev_xy
    for(int i=0; i<8; i++){
          prev_x[i] = -1;
        prev_y[i] = -1;
    }

    while (x <= y) {
        // Draw points in all 8 octants
           int point_x[8];
           int point_y[8];
        point_x[0] = circle.x + x;
        point_y[0] = circle.y + y;
        point_x[1] = circle.x + y;
        point_y[1] = circle.y + x;
        point_x[2] = circle.x - y;
        point_y[2] = circle.y + x;
        point_x[3] = circle.x - x;
        point_y[3] = circle.y + y;
        point_x[4] = circle.x - x;
        point_y[4] = circle.y - y;
        point_x[5] = circle.x - y;
        point_y[5] = circle.y - x;
        point_x[6] = circle.x + y;
        point_y[6] = circle.y - x;
        point_x[7] = circle.x + x;
        point_y[7] = circle.y - y;

      for(int i=0; i<8; i++){
        if(prev_x[i] != -1){
        SDL_RenderDrawLine(renderer, prev_x[i], prev_y[i], point_x[i], point_y[i]);
           }
        prev_x[i] = point_x[i];
        prev_y[i] = point_y[i];

      }

        // Error term update
            // Update d
        if(d<0){
            d += 4*x+6;
        } else {
            d += 4 * (x - y) + 10;
            y--;
        }
        x++;


    }

}

int body_collision(body *b1, body *b2){
    if(b1->rect.x < b2->rect.x + b2->rect.w &&
       b1->rect.x + b1->rect.w > b2->rect.x &&
       b1->rect.y < b2->rect.y + b2->rect.h &&
       b1->rect.y + b1->rect.h > b2->rect.y){
        return 1;
    }
    return 0;
}

void simple_resolve_collision(body *b1, body *b2){
    float tempX = b1->Xspeed;
    float tempY = b1->Yspeed;
    b1->Xspeed = b2->Xspeed;
    b1->Yspeed = b2->Yspeed;
    b2->Xspeed = tempX;
    b2->Yspeed = tempY;
    return;
}

void update_body(body *b){
    b->x += b->Xspeed;
    b->y += b->Yspeed;
    b->rect.x = (int)b->x;
    b->rect.y = (int)b->y;
    return;
}

void draw_body(SDL_Renderer *renderer, body *b){
    SDL_SetRenderDrawColor(renderer, b->color.r, b->color.g, b->color.b, b->color.a);
    SDL_RenderFillRect(renderer, &b->rect);
    return;
}


body *create_body(int *numBodys_ptr, int x, int y, int width, int height, float Xspeed, float Yspeed,float mass, SDL_Color color){
    body *b = malloc(sizeof(body));
    b->rect.x = x;
    b->rect.y = y;
    b->rect.w = width;
    b->rect.h = height;
    b->Xspeed = Xspeed;
    b->Yspeed = Yspeed;
    b->x = x;
    b->y = y;
    b->mass = mass;
    b->color = color;
    (*numBodys_ptr)++;
    return b;
}



void draw_rect(SDL_Renderer *renderer, SDL_Rect *rect, SDL_Color *color) {
    SDL_SetRenderDrawColor(renderer, color->r, color->g, color->b, color->a);
    SDL_RenderFillRect(renderer, rect);
    return;
}





int main(int argc, char *argv[]) {
    int numBodys = 0;
    int running = true;
    body *bodys[100];
    bodys[0] = create_body(&numBodys,50,50,10,10,1,0,1,RED);
    bodys[1] = create_body(&numBodys,200,50,20,20,-1,0,10,GREEN);
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL initialization failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("SDL2 Base", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_SHOWN);
    if (!window) {
        printf("Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    while(running) {
        Uint32 frameStart = SDL_GetTicks();

        SDL_Event event;
        while(SDL_PollEvent(&event)) {
           switch (event.type) {
            case SDL_QUIT:
                    running = 0;
                    break;
                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym) {
                        case SDLK_a:
                            printf("Key 'A' pressed\n");
                            break;
                    }
            }
        }
        // Set background color to white
        set_color(renderer, WHITE);
        SDL_RenderClear(renderer);

        draw_circle_octants(renderer, (circle){50,50,50},BLUE);

        for(int i = 0; i < numBodys; i++){
            for(int j = 0; j < numBodys; j++){
                if(i != j){
                    if(body_collision(bodys[i], bodys[j])){
                        calculate_vector_collision(bodys[i], bodys[j]);
                    }
                }
            }
            update_body(bodys[i]);
            draw_body(renderer, bodys[i]);
        }
        SDL_RenderPresent(renderer);

        // Frame limiting
        int frameTime = SDL_GetTicks() - frameStart;
        int delayTime = 1000 / 60 - frameTime;
        if (delayTime > 0) {
            SDL_Delay(delayTime);
        }
    }

    for(int i = 0; i < numBodys; i++){
        free(bodys[i]);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
