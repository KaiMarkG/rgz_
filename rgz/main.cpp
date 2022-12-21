#include<stdio.h>
#include<iostream>
#include"graph.h"
#include<unordered_map>
#include<queue>
#include<stack>
#include<SFML/Graphics.hpp>
#include<fstream>
#include<string>
#include <random>
#include "ModelStructures.h"
#include "Pathfinder_PRM.h"


using namespace std;
using namespace sf;

std::ostream& operator<< (std::ostream& out, const std::vector<int>& v) {
    for (auto e : v) out << e << ' ';
    return out;
}

std::istream& operator>> (std::istream& in, std::vector<int>& v) {
    for (int i = 0; i < v.size(); i++) in >> v[i];
    return in;
}

void save_data(std::ostream& fout,float robotRadius,
    std::vector<point>& target,
    std::vector<CircleObst>& obstacles,
    std::vector<int>& path,
    std::map<int,point>& nXY,
    Graph& g) {
    fout << robotRadius << endl;
    fout << obstacles.size() << endl;
    for (int i = 0; i < obstacles.size(); ++i) {
        CircleObst& obs = obstacles[i];
        fout << obs.x << ' ' << obs.y << ' ' << obs.r << std::endl;
    }
    fout << path.size() << endl;
    fout << path << endl;
    fout << target.size() << endl;
    for (auto p : target)
        fout << p.x << ' ' << p.y << endl;
    fout << nXY.size() << endl;
    for (auto p : nXY)
        fout << p.first<< ' ' << p.second.x << ' ' << p.second.y << endl;
    fout << g;
}

void load_data(std::istream& fin, float &robotRadius,
    std::vector<point>& target,
    std::vector<CircleObst>& obstacles,
    std::vector<int>& path,
    std::map<int,point>& nXY,
    Graph& g) {
    int n = 0;
    fin >> robotRadius;
    fin >> n;
    obstacles.resize(n);
    for (int i = 0; i < obstacles.size(); ++i) {
        CircleObst& obs = obstacles[i];
        fin >> obs.x >> obs.y >> obs.r;
    }
    fin >> n;
    path.resize(n);
    fin >> path;
    fin >> n;
    target.resize(n);
    for (auto& p : target)
        fin >> p.x >> p.y;
    fin >> n;
    for (int i = 0; i < n; i++) {
        int v;
        point p;
        fin >> v >> p.x >> p.y;
        nXY[v] = p;
    }
    fin >> g;
}


int main() {
    //===  CHANGEABLE PARAMETERS ===
    // Program window size
    const float X = 800, Y = 600;
    // the number of points in the construction PRM
    int n = 300;
    // the maximum number of attempts to build a PRM for which it is possible to find a path
    int maxiter = 100;
    //===============================


    unsigned int ttime=0;
    vector<CircleObst> obstacles; 
    vector<int> path;
    vector <point> targets;
    float robot_radius;
    int inputCounter = 0;
    bool obstacles_set = false;
    bool calculations_done = false;
    bool error = false;
    std::map<int, point> nodeXY;
    Graph g;
    
    ContextSettings settings;
    settings.antialiasingLevel = 5;
   
    RenderWindow window(VideoMode(X, Y), "PathFinder",Style::Default,settings);
    
    Text mainText;
    Font font;
    if (!font.loadFromFile("BOOKMANIA-REGULAR SK7_1.ttf"))
    {
        return EXIT_FAILURE;
    }
    mainText.setFont(font);
    mainText.setString("Click to set robot position or press L to load scene.");
    mainText.setPosition(10.f, 5.f);
    mainText.setFillColor(Color::Black);

   while (window.isOpen())
   {
       Event event;
       int mouseX = sf::Mouse::getPosition(window).x,
           mouseY = sf::Mouse::getPosition(window).y;
       while (window.pollEvent(event)) {
           //Closing the window when clicking on the cross
           if (event.type == Event::Closed) window.close();

           //The robot moves along the path when the left and right arrows are pressed, if the calculations are performed
           if (event.type == Event::KeyPressed && event.key.code == Keyboard::Right && ttime < path.size() - 1 && calculations_done)
               ttime += 1;
           if (event.type == Event::KeyPressed && event.key.code == Keyboard::Left && ttime > 0 && calculations_done)
               ttime -= 1;

           //Saving a scene to a file when pressing "S" if calculations are performed
           if (event.type == Event::KeyPressed && event.key.code == Keyboard::S && calculations_done) {
               ofstream fout;
               fout.open("SavedScene.txt");
               if (fout.is_open()) {
                   save_data(fout, robot_radius, targets, obstacles, path, nodeXY, g);
                   fout.close();
                   mainText.setString("Scene saved.");
               }
           }

           //Loading when pressing "L" if no data has been entered
           if (event.type == Event::KeyPressed && event.key.code == Keyboard::L && inputCounter == 0) {
               ifstream fin("SavedScene.txt");
               if (fin.is_open()) {
                   String line;
                   load_data(fin, robot_radius, targets, obstacles, path, nodeXY, g);
                   inputCounter = 1 + obstacles.size() * 2 + targets.size();
                   obstacles_set = true;
                   calculations_done = true;
                   mainText.setString("Scene loaded.");
               }
               else mainText.setString("File could not be opened. Click to set robot position.");
           }
           
           //Entering data with the mouse
           if (event.type == Event::MouseButtonPressed && !calculations_done) {
               
               switch (inputCounter) {
               case 0:
                   targets.push_back(point(mouseX, mouseY));
                   mainText.setString("Click to set robot radius.");
                   ++inputCounter;
                   break;
               case 1:
                   robot_radius = sqrt(pow(targets[0].x - mouseX, 2) + pow(targets[0].y - mouseY, 2));
                   ++inputCounter;
                   mainText.setString("Click to set goal position.");
                   break;
               case 2:
                   targets.push_back(point(mouseX, mouseY));
                   ++inputCounter;
                   mainText.setString("Click to set obstacle position or press enter.");
                   break;
               default:
                   if (!obstacles_set) {
                       if (inputCounter % 2 == 1) {
                           obstacles.push_back(CircleObst(mouseX, mouseY, 0.f));
                           ++inputCounter;
                           mainText.setString("Click to set obstacle radius.");
                       }
                       else {
                           CircleObst& obs = obstacles.back();
                           obs.r = (sqrt(pow(obs.x - mouseX, 2)
                               + pow(obs.y - mouseY, 2)));
                           ++inputCounter;
                           mainText.setString("Click to set obstacle position or press enter.");
                       }
                   }
                   else {
                       targets.push_back(point(mouseX, mouseY));
                   }
                   break;
               }

           }
           
           //Starting calculations or switching to the placement of additional points
           if (event.type == Event::KeyPressed && event.key.code == Keyboard::Enter
               && inputCounter > 2 && inputCounter % 2 == 1 && !calculations_done) {
               if (!obstacles_set) {
                   obstacles_set = true;
                   mainText.setString("Click to set point or press enter to calculate.");
               }
               else {
                   int iter = 0;
                   do {
                       Pathfinder_PRM solver(point(X, Y), targets, obstacles, robot_radius, n);
                       solver.find_path();
                       g = solver.get_graph();
                       path = solver.get_final();
                       nodeXY = solver.get_vertex_pos();
                       calculations_done = solver.solved();
                       iter++;
                   } while (!calculations_done && iter < maxiter);
                   if (!calculations_done) {
                       mainText.setString("Cannot calculate path. Input another data.");
                       inputCounter = 0;
                       obstacles_set = calculations_done = false;
                       targets.clear();
                       obstacles.clear();
                   }
                   else {
                       mainText.setString("Use \"<-\", \"->\" to move. S to save scene.");
                   }
               }
               
           }
       }
       window.clear(Color(255, 250, 250));

       //drawing a graph
       if (calculations_done) {
           for (auto v : g.get_vertices()) {
               for (auto e : g.get_adjacent_edges(v)) {
                   Vertex lineV[] = {
                           Vertex(Vector2f(nodeXY[e.first].x,nodeXY[e.first].y),Color(20,20,20,10)),
                           Vertex(Vector2f(nodeXY[v].x ,nodeXY[v].y),Color(20,20,20,10))
                    };
                    window.draw(lineV, 2, Lines);
               }
           }
       }

       //drawing a robot
       if (inputCounter > 0) {

           //real-time drawing the robot radius when entering
           if (inputCounter == 1) {
               robot_radius = sqrt(pow(targets[0].x - mouseX, 2) + pow(targets[0].y - mouseY, 2));
           }
           CircleShape robot(robot_radius);
           robot.setOrigin(robot_radius,robot_radius);
           robot.setFillColor(Color(0, 255, 230));
           if (calculations_done) robot.setPosition(nodeXY[path.at(ttime)].x, nodeXY[path.at(ttime)].y);
           else robot.setPosition(targets[0].x, targets[0].y);
           window.draw(robot);
           CircleShape robotcore(1.f);
           robotcore.setOrigin(1.f, 1.f);
           robotcore.setFillColor(Color(255, 75, 75));
           if (calculations_done) robotcore.setPosition(nodeXY[path.at(ttime)].x, nodeXY[path.at(ttime)].y);
           else robotcore.setPosition(targets[0].x, targets[0].y);
           window.draw(robotcore);

       }

       //drawing obstacles
       if (inputCounter > 2) { 
           for (int i = 0; i < obstacles.size(); ++i) {
               CircleShape cwall(obstacles[i].r);
               cwall.setOrigin(obstacles[i].r, obstacles[i].r);
               cwall.setFillColor(Color(255, 81, 59));
               cwall.setPosition(obstacles[i].x, obstacles[i].y);
               window.draw(cwall);
           }
           if (inputCounter % 2 == 0 && !obstacles_set) {
               CircleObst& obs = obstacles.back();
               float curr = sqrt(pow(obs.x - mouseX, 2) + pow(obs.y-mouseY, 2));
               CircleShape cwall(curr);
               cwall.setOrigin(curr, curr);
               cwall.setFillColor(Color(255, 81, 59));
               cwall.setPosition(obs.x, obs.y);
               window.draw(cwall);
           }
       }

       // drawing additional points
       if (obstacles_set) {
           for (int i = 2; i < targets.size(); i++) {
               CircleShape p(5.f);
               p.setOrigin(5.f, 5.f);
               p.setFillColor(Color(200, 30, 75));
               p.setPosition(targets[i].x, targets[i].y);
               window.draw(p);
           }
       }

       //drawing a path
       if (calculations_done) { 
           for (int i = 0; i < path.size() - 1; ++i) {
               int cur = path.at(i);
               int next = path.at(i + 1);
               Vertex pathline[] = {
               Vertex(Vector2f(nodeXY[cur].x, nodeXY[cur].y),Color(255, 175, 0)),
               Vertex(Vector2f(nodeXY[next].x, nodeXY[next].y),Color(255, 175, 0))
               };
               window.draw(pathline, 2, Lines);
           }
       }

       //drawing a goal
       if (inputCounter > 2) { 
           CircleShape goal(5.f);
           goal.setOrigin(5.f, 5.f);
           goal.setFillColor(Color(31, 255, 87));
           goal.setPosition(targets[1].x, targets[1].y);
           window.draw(goal);
       }

       // showing a drawn window
       window.draw(mainText);
        window.display();
    }

    return 0;
}
