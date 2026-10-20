#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>
#include <tuple>
#include <string>

#define PI 3.141592654
#define LINKLENGTH_CELLS 10
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y)*(XSIZE) + (X))

using namespace std;

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	*pX = (int)(x);
	if (*pX < 0) *pX = 0;
	if (*pX >= x_size) *pX = x_size - 1;

	*pY = (int)(y);
	if (*pY < 0) *pY = 0;
	if (*pY >= y_size) *pY = y_size - 1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;
	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex) {
		params->Y1=p1x; params->X1=p1y;
		params->Y2=p2x; params->X2=p2y;
	} else {
		params->X1=p1x; params->Y1=p1y;
		params->X2=p2x; params->Y2=p2y;
	}

	if ((p2x - p1x) * (p2y - p1y) < 0) {
		params->Flipped = 1;
		params->Y1 = -params->Y1;
		params->Y2 = -params->Y2;
	} else {
		params->Flipped = 0;
	}

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;
	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;
	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped) *x = -*x;
    } else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped) *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) return 0;
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double* map, int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	if(x0 < 0 || x0 >= x_size || x1 < 0 || x1 >= x_size ||
	   y0 < 0 || y0 >= y_size || y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size) {
    double x0, y0, x1, y1;
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(int i = 0; i < numofDOFs; i++){
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

tuple<double*, int, int> loadMap(string filepath) {
	FILE *f = fopen(filepath.c_str(), "r");
	if (!f) throw runtime_error("Failed to open map: " + filepath);
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) throw runtime_error("Invalid map header");
	double* map = new double[height*width];
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) throw runtime_error("Invalid map body");
			} while (isspace(c));
			map[y + x*width] = (c != '0') ? 1.0 : 0.0;
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

double distConfig(const vector<double>& q1, const vector<double>& q2) {
    double sum = 0.0;
    for (size_t i = 0; i < q1.size(); ++i) {
        double d = fmod(q2[i] - q1[i], 2.0 * PI);
        if (d > PI) d -= 2.0 * PI;
        if (d < -PI) d += 2.0 * PI;
        sum += d * d;
    }
    return sqrt(sum);
}

void generatePairs(const string& mapFile, int numDOFs, int numPairs, const string& outFile, unsigned int seed) {
    double* map;
    int x_size, y_size;
    tie(map, x_size, y_size) = loadMap(mapFile);

    mt19937 rng(seed);
    uniform_real_distribution<double> distAngle(0.0, 2.0 * PI);

    ofstream out(outFile);
    out << "trial,numDOFs,start,goal" << endl;

    for (int p = 0; p < numPairs; ++p) {
        vector<double> start(numDOFs);
        vector<double> goal(numDOFs);

        // Sample valid start
        while (true) {
            for (int i = 0; i < numDOFs; ++i) start[i] = distAngle(rng);
            if (IsValidArmConfiguration(start.data(), numDOFs, map, x_size, y_size)) break;
        }

        // Sample valid goal with min separation
        while (true) {
            for (int i = 0; i < numDOFs; ++i) goal[i] = distAngle(rng);
            if (IsValidArmConfiguration(goal.data(), numDOFs, map, x_size, y_size)) {
                if (distConfig(start, goal) >= 1.5) break;
            }
        }

        out << p << "," << numDOFs << ",";
        for (int i = 0; i < numDOFs; ++i) {
            out << start[i] << (i + 1 < numDOFs ? ":" : ",");
        }
        for (int i = 0; i < numDOFs; ++i) {
            out << goal[i] << (i + 1 < numDOFs ? ":" : "\n");
        }
    }
    out.close();
    delete[] map;
    cout << "Generated " << numPairs << " pairs for " << mapFile << " into " << outFile << endl;
}

int main(int argc, char** argv) {
    if (argc >= 5) {
        string mapFile = argv[1];
        int numDOFs = stoi(argv[2]);
        int numPairs = stoi(argv[3]);
        string outFile = argv[4];
        unsigned int seed = (argc >= 6) ? stoi(argv[5]) : 16782;
        generatePairs(mapFile, numDOFs, numPairs, outFile, seed);
        return 0;
    }
    generatePairs("../maps/map2.txt", 3, 20, "../output/benchmark_pairs_map2.csv", 16782);
    generatePairs("../maps/map1.txt", 5, 20, "../output/benchmark_pairs_map1.csv", 16782);
    generatePairs("../maps/map3.txt", 4, 20, "../output/benchmark_pairs_map3.csv", 16782);
    generatePairs("../maps/map4.txt", 5, 20, "../output/benchmark_pairs_map4.csv", 16782);
    generatePairs("../maps/map5.txt", 4, 20, "../output/benchmark_pairs_map5.csv", 16782);
    return 0;
}
