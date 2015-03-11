#include <fstream>
#include <assert.h>
#include <sstream>
#include <iostream>
#include "maze.h"
#include <iomanip>
#include <sys/resource.h>
#include <math.h>
#include "graphics.h"
#include <cstdlib> 
#include <cstdio> 
#include <vector>

using namespace std;

int **G;                    //the whole graph G[][]
Point* upperright;          //the upper right point of the whole graph
Path terms;                 //terminals to be connected
ObstacleVector obs;         //obstacles
PathVector completeGraph;   
PathVector mst;             //minimum spanning tree
int len;

int main(int argc, char *argv[]) {
    
    cout << "Begin ..." << endl;
    init(argv[1]);//get the file name

    /*
    //part 1: finish prim() of MST
    prim();

    //print out MST
    
    cout << "Printing out MST ..." << endl;
    for (PathVector::iterator spi = mst.begin(); spi != mst.end(); spi++) {
        Path* sp = (*spi);
        for (Path::iterator pi = sp->begin(); pi != sp->end(); pi++) {
            G[(*pi)->x][(*pi)->y] = 1;
        }
    }
    
    //calculate wirelength
    len = 0;
    for (int i = 1; i <= upperright->x; i++)
        for (int j = 1; j <= upperright->y; j++)
            if(G[i][j] == 1)
                len++;
   
    print_graph();
    // write function to dump you results into a file called 
    // "mst_result.txt"
    // the content is the same as the printed content in print_graph();
    
    //String file_name = "mst_result.txt";
    //dump_to_file(file_name);
    clean();
    */ 

    // part 2: design your own routing algorithm
    // the following is a skeleton of a maze routing
    // the keep funtion maze() is not shown here
    
    double beginTime = mazegetTime();//begin time
    //find shortest path for each pair of terminals
    //And we calculate every two of the terminals' distance
    int termNum = terms.size();
    //termNum is the Num of Terminals
    for (int i = 0; i < termNum; i++)
        for (int j = i+1; j < termNum; j++) {
            Path* path = maze(terms[i], terms[j]);
            terms[i]->paths->push_back(path);
            terms[j]->paths->push_back(path);
            completeGraph.push_back(path);//add path at the end 
            cout << "Printing out graph for one shortest path ..." << endl;
            print_graph();
            print_path(path); 
            clean();
        }

    double endTime = mazegetTime();

    //print out complete graph
    cout << "Printing out complete graph ..." << endl;
    for (PathVector::iterator spi = completeGraph.begin(); spi != completeGraph.end(); spi++) {
        Path* sp = (*spi);
        for (Path::iterator pi = sp->begin(); pi != sp->end(); pi++) {
            G[(*pi)->x][(*pi)->y] = 1;
        }
    }
    print_graph();
    //calculate wirelength
    len = 0;
    for (int i = 1; i <= upperright->x; i++)
        for (int j = 1; j <= upperright->y; j++)
            if(G[i][j] == 1)
                len++;
    //print_graph();
    clean();
    

    cout << "Total wirelength:" << len << endl;
    cout << "Total runtime:" << endTime - beginTime << " usec." << endl;

    cout << "Finished." << endl;

    //draw();
}

 /**
  * Read file
  * @param fn file to read
  */
void init(char *fn) {
    ifstream in;
    in.open(fn);
    if (!in.is_open()) {
        cerr << "Could not open file " << fn << endl;
        assert(0);
    }
    string line, word;

    while(true) {
        getline(in, line);
        if (in.eof()) break;
        if (line.length() < 2) continue; // empty line, skip
        if (line.c_str()[0] == '#') continue;

        istringstream is(line);
        // get token
        is >> word;
        if(!word.compare("grid")) {
            is >> word;
            upperright = new Point();
            sscanf(word.c_str(), "(%d,%d)", &upperright->x, &upperright->y);
            cout << "grid (" << upperright->x << "," << upperright->y << ")" << endl;
        }
        if(!word.compare("term")) {
            is >> word;
            Point* tp = new Point();
            sscanf(word.c_str(), "(%d,%d)", &tp->x, &tp->y);
            cout << "term (" << tp->x << "," << tp->y << ")" << endl;
            terms.push_back(tp);
        }
        if(!word.compare("obs")) {
            is >> word;
            Point* tp1 = new Point();
            Point* tp2 = new Point();
            sscanf(word.c_str(), "(%d,%d)", &tp1->x, &tp1->y);
            cout << "obs (" << tp1->x << "," << tp1->y << ")";
            is >> word;
            sscanf(word.c_str(), "(%d,%d)", &tp2->x, &tp2->y);
            cout << " (" << tp2->x << "," << tp2->y << ")" << endl;
            Obstacle* to = new Obstacle(tp1,tp2);
            obs.push_back(to);
        }

    }
    in.close();

    G = new int*[upperright->x + 2];
    //set boundary
    for (int i = 0; i < upperright->x + 2; i++) {
        G[i] = new int[upperright->y + 2];
    }

    clean();
    
    for ( int i = 0; i < upperright->x + 2; i++) {
        G[i][0] = -1;
        G[i][upperright->y + 1] = -1;
    }

    for ( int i = 0; i < upperright->y + 2; i++) {
        G[0][i] = -1;
        G[upperright->x + 1][i] = -1;
    }

    //set obstacles
    for(ObstacleVector::iterator oi = obs.begin(); oi != obs.end();
        oi++) {
        for (int i = (*oi)->ll->x; i <= (*oi)->ur->x; i++)
            for (int j = (*oi)->ll->y; j <= (*oi)->ur->y; j++)
                G[i][j] = -1;

    }

    for(Path::iterator tp = terms.begin(); tp != terms.end(); tp++) {
            (*tp)->paths = new PathVector();
    }
}

Path* maze(Point* s, Point* t) {
    G[s->x][s->y] = 0;//source point
    G[t->x][t->y] = MAX - 1;//end point
    Path plist;
    Path nlist;
    plist.push_back(s);
    nlist.clear();
    int x[4];//x,y to give step move
    int y[4];
    x[0] = 1;  y[0] = 0;  //right
    x[1] = 0;  y[1] = 1;  //up
    x[2] = -1; y[2] = 0;  //left
    x[3] = 0;  y[3] = -1; //down
    int dis = 1;
    bool path_exists = false;
    while(plist.size() && !path_exists){
        for (Path::iterator pi = plist.begin(); 
            pi != plist.end() && !path_exists; pi++) {
            for (int i = 0; i < 4; i++) {
                Point* curP = new Point((*pi)->x+x[i], (*pi)->y+y[i]);
                if (G[curP->x][curP->y] == -1)
                    continue;
                if (G[curP->x][curP->y] > dis) {
                    G[curP->x][curP->y] = dis;
                    nlist.push_back(curP);
                }    
                if ((curP->x == t->x) && (curP->y == t->y)) {
                    path_exists = true;
                    break;
                }
            }
        }
        dis++;
        plist.clear();
        plist.insert(plist.begin(), nlist.begin(), nlist.end());
        nlist.clear();
    }     
    if (path_exists)
        return retrace(s, t);
    else {
        cout << "The path does not exist for (" << s->x << "," 
             << s->y << ") and (" << t->x << "," << t->y << ")." << endl;
        exit(0);        
    } 
}

Path* retrace(Point* s, Point* t) {
    Path* path = new Path();
    Point* curP = t;
    path->push_back(curP);
    int status = 0;
    int tx = 0;
    int ty = 0;
    while (true) {
        switch(status) {
            case 0: tx = curP->x + 1; ty = curP->y;     break; //right
            case 1: tx = curP->x;     ty = curP->y + 1; break; //up
            case 2: tx = curP->x - 1; ty = curP->y;     break; //left
            case 3: tx = curP->x;     ty = curP->y - 1; break; //down
        }
        if ((tx == s->x) && (ty == s->y)) {
            path->push_back(s);
            break;
        }
        if (G[tx][ty] == (G[curP->x][curP->y]-1)) {
            curP = new Point(tx, ty);
            path->push_back(curP);
        }
        else {
            status = (status + 1) % 4;
        }
    }
    return path;
}

void print_path(Path* path) {
    for (Path::iterator pi = path->begin(); pi != path->end(); pi++) {
        cout << "(" << (*pi)->x << "," << (*pi)->y << ") " << endl;
    }
}

void print_graph() {
    for (int j = upperright->y; j > 0; j--) {
        for (int i = 1; i <= upperright->x; i++) {
            if(G[i][j] == MAX)
                cout << " N | ";
            else
                cout << setw(2) << G[i][j] << " | ";
        }
        cout << endl;
    }
}
/**
 * to make G[][] become all 65535 when not obs(-1)
 */
void clean() {
    for (int i = 1; i <= upperright->x; i++)
        for (int j = 1; j <= upperright->y; j++)
            if (G[i][j] != -1)
                G[i][j] = MAX;
}

/**
 * difficult to understand
 */
void prim() {
    PointSet plist;
    PathSet ps;
    Point* curP = terms[0];//to let curP's beginning be the terminal
    for (int i = 0; i < terms.size() - 1; i++) {//for loop all terminals
        plist.insert(curP);
        for (PathVector::iterator spi = curP->paths->begin(); 
            spi != curP->paths->end(); spi++) {
            ps.insert((*spi));
        }
        Path* path = *ps.begin();
        //delete circle
        while (plist.find((*path)[0]) != plist.end() 
            && plist.find((*path)[path->size()-1]) != plist.end()) {
            ps.erase(ps.begin());
            path = *ps.begin();
        }
        if (plist.find((*path)[0]) == plist.end())
            curP = (*path)[0];
        else 
            curP = (*path)[path->size()-1];
        mst.push_back((*ps.begin()));
        ps.erase(ps.begin());
    } 
}

void draw() {
/* initialize display */
 init_graphics("Multiple terminals routing");


 init_world (0.,0.,1000.,1000.);
 //update_message("Interactive graphics example number 2!.");
 //create_button ("Window", "Next", new_button_func);
 drawscreen(); 
 event_loop(button_press, drawscreen);    
    
}

static void drawscreen (void) {

/* redrawing routine for still pictures.  Graphics package calls  *
 * this routine to do redrawing after the user changes the window *
 * in any way.                                                    */

 clearscreen();  /* Should be first line of all drawscreens */
 setfontsize (10);
 setlinestyle (SOLID);
 setlinewidth (0);
 setcolor (BLACK);
 int windowsize=WINDOWSIZE;
 int gridsize=(windowsize/upperright->x > windowsize/upperright->y)?
        (windowsize/upperright->y*.8):(windowsize/upperright->x*.8);

 //grid
 for (int i = 0; i < upperright->x; i++) {
    drawline(gridsize+i*gridsize, windowsize-gridsize, gridsize+i*gridsize, windowsize-upperright->y*gridsize);
 }

 for (int i = 0; i < upperright->y; i++) {
    drawline(gridsize, windowsize-gridsize-i*gridsize, upperright->x*gridsize, windowsize-gridsize-i*gridsize);
 }

 //obstacles
 for (ObstacleVector::iterator oi = obs.begin(); oi != obs.end(); oi++) {
    setcolor(RED);
    fillrect((*oi)->ll->x*gridsize, windowsize-(*oi)->ll->y*gridsize, (*oi)->ur->x*gridsize+gridsize, windowsize-(*oi)->ur->y*gridsize-gridsize);
 }   
 
 //MST
 for (PathVector::iterator spi = mst.begin(); spi != mst.end(); spi++) {
    Path* path = *spi;
    for (Path::iterator pi = path->begin(); pi != path->end(); pi++) {
        setcolor(BLUE);
        fillrect((*pi)->x*gridsize, windowsize-(*pi)->y*gridsize, (*pi)->x*gridsize+gridsize, windowsize-(*pi)->y*gridsize-gridsize);
        flushinput();
    }
 }

 //terminals
 for (Path::iterator pi = terms.begin(); pi != terms.end(); pi++) {
    setcolor(YELLOW);
    fillrect((*pi)->x*gridsize, windowsize-(*pi)->y*gridsize, (*pi)->x*gridsize+gridsize, windowsize-(*pi)->y*gridsize-gridsize);
    flushinput();
    //delay();
 } 

}

/**
 * Use it to delay
 */
static void delay (void) {

/* A simple delay routine for animation. */

 int i, j, k, sum;

 sum = 0;
 for (i=0;i<50000;i++) 
    for (j=0;j<i;j++)
       for (k=0;k<30;k++) 
          sum = sum + i+j-k; 
}

static void button_press (float x, float y) {

/* Called whenever event_loop gets a button press in the graphics *
 * area.  Allows the user to do whatever he/she wants with button *
 * clicks.                                                        */
 int windowsize=WINDOWSIZE;
 int gridsize=(windowsize/upperright->x > windowsize/upperright->y)?
        (windowsize/upperright->y*.8):(windowsize/upperright->x*.8);
 int ax = int(x) / gridsize;
 int ay = (windowsize - int(y)) / gridsize;

 printf("User clicked a button at grid (%d,%d)\n", ax, ay);
}

static void new_button_func (void (*drawscreen_ptr) (void)) {

 printf ("You pressed the new button!\n");
 setcolor (MAGENTA);
 setfontsize (12);
 drawtext (500., 500., "You pressed the new button!", 10000.);
}

/**
 * Use it to show system time
 * @return system time in us
 */
double mazegetTime() {
    struct rusage r;
    getrusage(RUSAGE_SELF, &r);
    
    return static_cast<double>(r.ru_utime.tv_sec)*1000000 + static_cast<double>(r.ru_utime.tv_usec);
}
