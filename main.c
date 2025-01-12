 
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <time.h>
const SDL_Color RED = {255,0,0,255};
const SDL_Color GREEN = {0,255,0,255};
const SDL_Color BLUE = {0,0,255,255};
const SDL_Color BLACK = {0,0,0,255};
const SDL_Color WHITE = {255,255,255,255};
const SDL_Color YELLOW = {255,255,0,255};
const SDL_Color CYAN = {0,255,255,255};
const SDL_Color MAGENTA = {255,0,255,255};
const SDL_Color ORANGE = {255,165,0,255};
const SDL_Color PURPLE = {128,0,128,255};
const SDL_Color PINK = {255,192,203,255};
const int WIDTH = 1920; //640
const int HEIGHT = 1080; //

// Structure to represent a 2D vector
typedef struct  {
    double x;
    double y;
} vector;


//int *numbodies_ptr, double x, double y, int radius, double Xspeed, double Yspeed,double mass, SDL_Color color
typedef struct { //body structure
    bool isAlive;
    double x,y; //position of the body
    double radius; //radius of the body
    double Xspeed, Yspeed; //speed of the body
    double mass;   //mass of the body

    SDL_Color color;

}body;

body* loadBodiesFromFile(const char *filename, int *numBodies, int *capacity) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return NULL; // Return NULL on failure
    }
    
    body *bodies = NULL;   // Start with no bodies, initial size is 0
    *numBodies = 0;        // initial number of bodies in file is 0
    char line[256];        // Buffer to hold a line from the file
           // How many elements the array can hold currently
    
    
    while (fgets(line, sizeof(line), file) != NULL) {
        
       
        bool isAlive = true;
        double radius, Xspeed, Yspeed, x, y, mass;
        Uint8 r, g, b, a;
        
        // Parse each line
        int itemsRead = sscanf(line, "%lf %lf %lf %lf %lf %lf  %hhu %hhu %hhu %hhu",
                                    &x, &y, &radius, &Xspeed, &Yspeed, &mass, &r, &g, &b, &a);
        printf("line read: %s\n", line);
       if(itemsRead != 10) {
           printf("Error parsing line: %s\n", line);
           continue; // skip to the next line
       }
       
       if(*capacity == *numBodies){
           // Reallocate when buffer is full
           *capacity = (*capacity == 0) ? 1 : *capacity * 2; // Grow capacity by factor of two if initial capacity is 0 or double the existing capacity
           bodies = realloc(bodies, *capacity * sizeof(body)); // Reallocate the memory buffer
           if(!bodies){
               perror("Error reallocating memory");
               fclose(file);
               return NULL; // Return NULL if realloc fails
           }
       }

       
       // Create a new body
       body newBody;
       newBody.isAlive = isAlive;
       newBody.radius = radius;
       newBody.Xspeed = Xspeed;
       newBody.Yspeed = Yspeed;
       newBody.x = x;
       newBody.y = y;
       newBody.mass = mass;
       newBody.color.r = r;
       newBody.color.g = g;
       newBody.color.b = b;
       newBody.color.a = a;
        
        bodies[*numBodies] = newBody; // add the newly created body to the buffer
        (*numBodies)++; // Increment the count of bodies
    }
    
    fclose(file); // close the file after it is done being read
    return bodies; // return the array of bodies
}

// Function to calculate the dot product of two vectors
double dot_product(vector v1, vector v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

// Function to calculate the magnitude (length) of a vector
double get_vector_magnitude(vector v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

double get_vector_angle(vector v) {
    return atan2(v.y, v.x);
}

//function to convert angle and magnitude to vector
vector angle_magnitude_to_vector(double angle, double magnitude){
    vector v;
    v.x = magnitude * cos(angle);
    v.y = magnitude * sin(angle);
    return v;
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

double get_points_angle(double x1, double y1, double x2, double y2) {
    double delta_y = y1 - y2;
    double delta_x = x1 - x2;
    return atan2(delta_y, delta_x);
}

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
    double norm_magnitude = get_vector_magnitude(normal);

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

int absorb_body(body **bodies, int *numBodies, int *capacity, int index1, int index2) {
    body *b1 = &(*bodies)[index1];
    body *b2 = &(*bodies)[index2];

    double new_mass = b1->mass + b2->mass;
    SDL_Color new_color;
    new_color.r = (b1->color.r + b2->color.r) / 2;
    new_color.g = (b1->color.g + b2->color.g) / 2;
    new_color.b = (b1->color.b + b2->color.b) / 2;
    new_color.a = (b1->color.a + b2->color.a) / 2;
    double new_radius = sqrt(pow(b1->radius, 2) + pow(b2->radius, 2));
    double new_x = (b1->x * b1->mass + b2->x * b2->mass) / new_mass;
    double new_y = (b1->y * b1->mass + b2->y * b2->mass) / new_mass;
    double new_Xspeed = (b1->Xspeed * b1->mass + b2->Xspeed * b2->mass) / new_mass;
    double new_Yspeed = (b1->Yspeed * b1->mass + b2->Yspeed * b2->mass) / new_mass;

    if (b1->mass > b2->mass) {
        b1->mass = new_mass;
        b1->radius = new_radius;
        b1->color = new_color;
        b1->x = new_x;
        b1->y = new_y;
        b1->Xspeed = new_Xspeed;
        b1->Yspeed = new_Yspeed;
    } else {
        b2->mass = new_mass;
        b2->radius = new_radius;
        b2->color = new_color;
        b2->x = new_x;
        b2->y = new_y;
        b2->Xspeed = new_Xspeed;
        b2->Yspeed = new_Yspeed;
    }

    // Remove the absorbed body from memory
    int absorbedIndex = (b1->mass > b2->mass) ? index2 : index1;
    for (int i = absorbedIndex; i < *numBodies - 1; i++) {
        (*bodies)[i] = (*bodies)[i + 1];
    }
    (*numBodies)--;

    // Reallocate memory to shrink the array
    *bodies = realloc(*bodies, (*capacity) * sizeof(body));
    if (!*bodies && *numBodies > 0) {
        perror("Error reallocating memory");
        return -1; // Return -1 if realloc fails
    }

    return 0;
}

void set_color(SDL_Renderer *renderer, SDL_Color color){
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    return;
}

void draw_circle_octants(SDL_Renderer* renderer, double body_x, double body_y, double radius) {
 
    int x = 0;
    int y = radius;
    int d = 1 - radius;
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
        point_x[0] = body_x + x;
        point_y[0] = body_y + y;
        point_x[1] = body_x + y;
        point_y[1] = body_y + x;
        point_x[2] = body_x - y;
        point_y[2] = body_y + x;
        point_x[3] = body_x - x;
        point_y[3] = body_y + y;
        point_x[4] = body_x - x;
        point_y[4] = body_y - y;
        point_x[5] = body_x - y;
        point_y[5] = body_y - x;
        point_x[6] = body_x + y;
        point_y[6] = body_y - x;
        point_x[7] = body_x + x;
        point_y[7] = body_y - y;

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

// Function to calculate collision and return overlap
int body_collision(body *b1, body *b2) {
    double distance = sqrt(pow(b1->x - b2->x, 2) + pow(b1->y - b2->y, 2));
    double overlap = b1->radius + b2->radius - distance;
    if (overlap > 0) {
        return 1;
    }
    return 0;
}

void simple_resolve_collision(body *b1, body *b2){
    double tempX = b1->Xspeed;
    double tempY = b1->Yspeed;
    b1->Xspeed = b2->Xspeed;
    b1->Yspeed = b2->Yspeed;
    b2->Xspeed = tempX;
    b2->Yspeed = tempY;
    return;
}

void update_body(int self, body *b[100], int numbodies){
   
    b[self]->x += b[self]->Xspeed;
    for(int i = 0; i < numbodies; i++){
        if(i != self && b[i]->isAlive){
            if(body_collision(b[self], b[i])){
                b[self]->x -= b[self]->Xspeed ;
                
            }
        }
    }
    b[self]->y += b[self]->Yspeed;
    for(int i = 0; i < numbodies; i++){
        if(i != self && b[i]->isAlive){
            if(body_collision(b[self], b[i])){
                b[self]->y -= b[self]->Yspeed;

            }
        }
    }
   
    return;
}

void draw_body(SDL_Renderer *renderer, body *b){
    SDL_SetRenderDrawColor(renderer, b->color.r, b->color.g, b->color.b, b->color.a);
    draw_circle_octants(renderer, b->x, b->y, b->radius);
    return;
}

void calculate_gravity(body *b1, body *b2){
    double distance = sqrt(pow(b1->x - b2->x, 2) + pow(b1->y - b2->y, 2));
    double force = 6.674 * pow(10,-11)* b1->mass * b2->mass / pow(distance,2);
    double angle = get_points_angle(b2->x, b2->y, b1->x, b1->y);
    vector force_vector = angle_magnitude_to_vector(angle, force);
    vector acceleration = scale_vector(force_vector, 1/b1->mass);
    vector velocity = scale_vector(acceleration, 0.01666667);
    b1->Xspeed += velocity.x;
    b1->Yspeed += velocity.y;
    return;
}

body* create_body(body *bodies, int *numBodies, int *capacity, double x, double y, double radius, double Xspeed, double Yspeed, double mass, SDL_Color color) {
    if (*capacity == *numBodies) {
        // Reallocate when buffer is full
        *capacity = (*capacity == 0) ? 1 : (*capacity * 2); // Grow capacity by factor of two if initial capacity is 0 or double the existing capacity
        bodies = realloc(bodies, (*capacity) * sizeof(body)); // Reallocate the memory buffer
        if (!bodies) {
            perror("Error reallocating memory");
            return NULL; // Return NULL if realloc fails
        }
    }

    // Create a new body
    body newBody;
    newBody.isAlive = true;
    newBody.x = x;
    newBody.y = y;
    newBody.radius = radius;
    newBody.Xspeed = Xspeed;
    newBody.Yspeed = Yspeed;
    newBody.mass = mass;
    newBody.color = color;

    bodies[*numBodies] = newBody; // Add the newly created body to the buffer
    (*numBodies)++; // Increment the count of bodies

    return bodies;
}


void draw_rect(SDL_Renderer *renderer, SDL_Rect *rect, SDL_Color *color) {
    SDL_SetRenderDrawColor(renderer, color->r, color->g, color->b, color->a);
    SDL_RenderFillRect(renderer, rect);
    return;
}





int main(int argc, char *argv[]) {
    int capacity = 0;
    int numbodies = 0;
    int running = true;
    int mouse_start_x = 0;
    int mouse_start_y = 0;
    int camera_x = 0;
    int camera_y = 0;
    int keys[4] = {0,0,0,0};
    vector mouse_vector;
    int mouse_x, mouse_y;
    srand(time(NULL));
    
    
    
    body *bodies = loadBodiesFromFile("bodies.txt", &numbodies,&capacity);

    //bodies[0] = create_body(&numbodies,300,300,20,-2,0,10e15,RED);
    //bodies[1] = create_body(&numbodies,100,300,20,0,-2,10e15,GREEN);
    // bodies[2] = create_body(&numbodies,100,100,20,2,0,10e15,BLUE);
    //bodies[3] = create_body(&numbodies,300,300,20,0,2,10e15,WHITE);
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL initialization failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("SDL2 Base", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        printf("Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    int xorign = WIDTH/2;
    int yorign = HEIGHT/2;
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    
  
    /*
    bodies[1] = create_body(&numbodies,xorign+550,yorign,4,0,-5.6 ,7e13,GREEN);
    bodies[2] = create_body(&numbodies,xorign,yorign,35,0,0,1e16,YELLOW);
    bodies[3] = create_body(&numbodies,xorign-500,yorign,15,0,4,1e14,CYAN);
    bodies[4] = create_body(&numbodies,xorign-550,yorign,4,0,5.6 ,7e13,MAGENTA);
    */
    while(running) {
        Uint32 frameStart = SDL_GetTicks();

        SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                running = 0;
                break;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_SPACE:
                         
                        for (int i = 0; i < numbodies; i++) {
                            bodies[i].x -= camera_x;
                            
                            bodies[i].y -= camera_y;
                            
                        }
                        camera_x = 0;
                        camera_y = 0;
                        break;

                    case SDLK_ESCAPE:
                        running = 0;
                        break;
                    case SDLK_w:
                        keys[0] = 1;
                        break;
                    case SDLK_a:
                        keys[1] = 1;
                        break;
                    case SDLK_s:
                        keys[2] = 1;
                        break;
                    case SDLK_d:
                        keys[3] = 1;
                        break;
                } 
            break;  
            case SDL_KEYUP:
                switch (event.key.keysym.sym) {
                    case SDLK_w:
                        keys[0] = 0;
                        break;
                    case SDLK_a:
                        keys[1] = 0;
                        break;
                    case SDLK_s:
                        keys[2] = 0;
                        break;
                    case SDLK_d:
                        keys[3] = 0;
                        break;
                }
            





                break;
            case SDL_MOUSEBUTTONDOWN:
                mouse_start_x = event.button.x;
                mouse_start_y = event.button.y;
                break;
            case SDL_MOUSEBUTTONUP:
                mouse_vector.x = -(event.button.x - mouse_start_x)/30;
                mouse_vector.y = -(event.button.y - mouse_start_y)/30;

                if(event.button.button == SDL_BUTTON_LEFT){
                    bodies = create_body(bodies, &numbodies, &capacity, event.button.x, event.button.y, 5, mouse_vector.x*2, mouse_vector.y*2, 1e12, BLUE);
                }else if(event.button.button == SDL_BUTTON_RIGHT){
                    bodies = create_body(bodies, &numbodies, &capacity, event.button.x, event.button.y, 15, mouse_vector.x, mouse_vector.y, 1e14, RED);
                }else if (event.button.button == SDL_BUTTON_MIDDLE){
                    for(int i = 0; i < 10; i++){
                            bodies = create_body(bodies, &numbodies, &capacity,
                             event.button.x+ rand() % (100 - -100 + 1), event.button.y + rand() % (100 - -100 + 1)
                             , 5, mouse_vector.x, mouse_vector.y, 1e8, GREEN);
                        }
                }
                break;
        }
    }       
        if(keys[0]){
            for(int i = 0; i < numbodies;i++){
                bodies[i].y += 5;
                camera_y += 5;
            }
        }
        if(keys[1]){
            for(int i = 0; i< numbodies;i++){
                bodies[i].x += 5;
                camera_x += 5;
            }
        }
        if(keys[2]){
            for(int i = 0; i< numbodies;i++){
                bodies[i].y += -5;
                camera_y -= 5;
            }
        }
        if(keys[3]){
            for(int i = 0; i< numbodies;i++){
                bodies[i].x += -5;
                camera_x -= 5;
            }
        }

        // Set background color to white
        set_color(renderer, BLACK);
        SDL_RenderClear(renderer);

        
        for(int i = 0; i < numbodies; i++){
            for(int j = 0; j < numbodies; j++){
                if(i != j && bodies[i].isAlive && bodies[j].isAlive){
                    if(body_collision(&bodies[i], &bodies[j])){
                        //calculate_vector_collision(bodies[i], bodies[j]);
                        absorb_body(&bodies, &numbodies, &capacity, i, j);
                    }else {
                        calculate_gravity(&bodies[i], &bodies[j]);
                    }
                }
            }
        }
        for(int i = 0; i < numbodies; i++){
            if (bodies[i].isAlive){
                //update_body(i, bodies, numbodies);
                bodies[i].x += bodies[i].Xspeed;
                bodies[i].y += bodies[i].Yspeed;
                draw_body(renderer, &bodies[i]);
            }
        }
        SDL_RenderPresent(renderer);

        // Frame limiting
        int frameTime = SDL_GetTicks() - frameStart;
        int delayTime = 1000 / 60 - frameTime;
        if (delayTime > 0) {
            SDL_Delay(delayTime);
        }
    }

    free(bodies);

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
