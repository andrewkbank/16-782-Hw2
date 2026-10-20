/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h>
#include <climits>
#include <cstdlib>
#include <chrono>
#include <queue>
#include <cmath>
#include <memory>
#include <unordered_set>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3
#define RRTSTAR_NR  4
#define RRTSTAR_LB  5
#define RRTSTAR_CO  6

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif


// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;
using std::pair;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

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
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

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
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

void planner(
    double* map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          SAMPLING-BASED PLANNING HELPERS                                          //
//                                                                                                                   //
//*******************************************************************************************************************//

inline double wrapAngle(double a) {
    double res = fmod(a, 2.0 * PI);
    if (res < 0.0) res += 2.0 * PI;
    return res;
}

inline double diffAngle(double a, double b) {
    double d = fmod(b - a, 2.0 * PI);
    if (d > PI) d -= 2.0 * PI;
    if (d < -PI) d += 2.0 * PI;
    return d;
}

inline double distAngle(double a, double b) {
    return fabs(diffAngle(a, b));
}

inline double distConfig(const vector<double>& q1, const vector<double>& q2) {
    double sum = 0.0;
    for (size_t i = 0; i < q1.size(); ++i) {
        double d = diffAngle(q1[i], q2[i]);
        sum += d * d;
    }
    return sqrt(sum);
}

vector<double> steerConfig(const vector<double>& from, const vector<double>& toward, double max_dist) {
    double d = distConfig(from, toward);
    if (d <= max_dist || d < 1e-9) {
        return toward;
    }
    double frac = max_dist / d;
    vector<double> res(from.size());
    for (size_t i = 0; i < from.size(); ++i) {
        double delta = diffAngle(from[i], toward[i]);
        res[i] = wrapAngle(from[i] + frac * delta);
    }
    return res;
}

bool isValidEdge(const vector<double>& q1, const vector<double>& q2,
                 int numofDOFs, double* map, int x_size, int y_size,
                 double step_res = PI / 20.0) {
    double max_diff = 0.0;
    vector<double> diffs(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) {
        diffs[i] = diffAngle(q1[i], q2[i]);
        double ad = fabs(diffs[i]);
        if (ad > max_diff) max_diff = ad;
    }
    int num_steps = static_cast<int>(ceil(max_diff / step_res));
    if (num_steps < 1) num_steps = 1;

    vector<double> sample(numofDOFs);
    for (int s = 1; s <= num_steps; ++s) {
        double t = static_cast<double>(s) / num_steps;
        for (int i = 0; i < numofDOFs; ++i) {
            if (s == num_steps) {
                sample[i] = q2[i];
            } else {
                sample[i] = wrapAngle(q1[i] + t * diffs[i]);
            }
        }
        if (!IsValidArmConfiguration(sample.data(), numofDOFs, map, x_size, y_size)) {
            return false;
        }
    }
    return true;
}

void generateOutputPlan(const vector<vector<double>>& waypoints,
                        int numofDOFs,
                        double*** plan,
                        int* planlength,
                        double step_res = PI / 20.0) {
    if (waypoints.empty()) {
        *plan = nullptr;
        *planlength = 0;
        return;
    }

    vector<vector<double>> densePath;
    densePath.push_back(waypoints[0]);

    for (size_t w = 0; w + 1 < waypoints.size(); ++w) {
        const auto& q1 = waypoints[w];
        const auto& q2 = waypoints[w + 1];

        double max_diff = 0.0;
        vector<double> diffs(numofDOFs);
        for (int i = 0; i < numofDOFs; ++i) {
            diffs[i] = diffAngle(q1[i], q2[i]);
            double ad = fabs(diffs[i]);
            if (ad > max_diff) max_diff = ad;
        }
        int num_steps = static_cast<int>(ceil(max_diff / step_res));
        if (num_steps < 1) num_steps = 1;

        vector<double> sample(numofDOFs);
        for (int s = 1; s <= num_steps; ++s) {
            double t = static_cast<double>(s) / num_steps;
            for (int i = 0; i < numofDOFs; ++i) {
                if (s == num_steps) {
                    sample[i] = q2[i];
                } else {
                    sample[i] = wrapAngle(q1[i] + t * diffs[i]);
                }
            }
            densePath.push_back(sample);
        }
    }

    int n = static_cast<int>(densePath.size());
    *plan = (double**) malloc(n * sizeof(double*));
    for (int i = 0; i < n; ++i) {
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; ++j) {
            (*plan)[i][j] = densePath[i][j];
        }
    }
    *planlength = n;
}

struct RRTNode {
    vector<double> config;
    int parent;
};

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    auto startTime = std::chrono::steady_clock::now();
    vector<double> startVec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> goalVec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    if (distConfig(startVec, goalVec) < 1e-4) {
        vector<vector<double>> single = {startVec, goalVec};
        generateOutputPlan(single, numofDOFs, plan, planlength);
        return;
    }

    vector<RRTNode> tree;
    tree.push_back({startVec, -1});

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> distAngleRng(0.0, 2.0 * PI);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    const double stepSize = 0.35;
    double goalBias = 0.08;
    const char* envBias = std::getenv("RRT_GOAL_BIAS");
    if (envBias != nullptr) {
        try {
            goalBias = std::stod(envBias);
        } catch (...) {}
    }
    int goalParent = -1;

    for (int iter = 0; iter < 40000; ++iter) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed > 4500) {
            break;
        }

        vector<double> q_rand(numofDOFs);
        if (dist01(rng) < goalBias) {
            q_rand = goalVec;
        } else {
            for (int i = 0; i < numofDOFs; ++i) {
                q_rand[i] = distAngleRng(rng);
            }
        }

        int nearestIdx = 0;
        double minDist = distConfig(tree[0].config, q_rand);
        for (size_t i = 1; i < tree.size(); ++i) {
            double d = distConfig(tree[i].config, q_rand);
            if (d < minDist) {
                minDist = d;
                nearestIdx = static_cast<int>(i);
            }
        }

        int currIdx = nearestIdx;
        for (int s = 0; s < 10; ++s) {
            vector<double> q_step = steerConfig(tree[currIdx].config, q_rand, stepSize);
            if (distConfig(tree[currIdx].config, q_step) < 1e-4) {
                break;
            }
            if (!isValidEdge(tree[currIdx].config, q_step, numofDOFs, map, x_size, y_size)) {
                break;
            }
            int newIdx = static_cast<int>(tree.size());
            tree.push_back({q_step, currIdx});
            currIdx = newIdx;

            if (distConfig(q_step, goalVec) <= 1.2) {
                if (isValidEdge(q_step, goalVec, numofDOFs, map, x_size, y_size)) {
                    goalParent = newIdx;
                    break;
                }
            }
            if (distConfig(q_step, q_rand) < 1e-4) {
                break;
            }
        }
        if (goalParent != -1) {
            break;
        }
    }

    printf("[STATS] vertices: %zu\n", tree.size());
    if (goalParent != -1) {
        vector<vector<double>> waypoints;
        waypoints.push_back(goalVec);
        int curr = goalParent;
        while (curr != -1) {
            waypoints.push_back(tree[curr].config);
            curr = tree[curr].parent;
        }
        std::reverse(waypoints.begin(), waypoints.end());
        waypoints.front() = startVec;
        waypoints.back() = goalVec;
        generateOutputPlan(waypoints, numofDOFs, plan, planlength);
    } else {
        planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
    }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

enum RRTExtendStatus { RRT_REACHED, RRT_ADVANCED, RRT_TRAPPED };

static RRTExtendStatus rrtConnectExtend(
    vector<RRTNode>& tree,
    const vector<double>& q_target,
    double stepSize,
    int numofDOFs,
    double* map,
    int x_size,
    int y_size,
    int& newIdxOut)
{
    newIdxOut = -1;
    int nearestIdx = 0;
    double minDist = distConfig(tree[0].config, q_target);
    for (size_t i = 1; i < tree.size(); ++i) {
        double d = distConfig(tree[i].config, q_target);
        if (d < minDist) {
            minDist = d;
            nearestIdx = static_cast<int>(i);
        }
    }

    vector<double> q_new = steerConfig(tree[nearestIdx].config, q_target, stepSize);
    if (!isValidEdge(tree[nearestIdx].config, q_new, numofDOFs, map, x_size, y_size)) {
        return RRT_TRAPPED;
    }

    newIdxOut = static_cast<int>(tree.size());
    tree.push_back({q_new, nearestIdx});

    if (distConfig(q_new, q_target) < 1e-4) {
        return RRT_REACHED;
    }
    return RRT_ADVANCED;
}

static RRTExtendStatus rrtConnectConnect(
    vector<RRTNode>& tree,
    const vector<double>& q_target,
    double stepSize,
    int numofDOFs,
    double* map,
    int x_size,
    int y_size,
    int& finalIdxOut)
{
    RRTExtendStatus status;
    finalIdxOut = -1;
    do {
        int newIdx = -1;
        status = rrtConnectExtend(tree, q_target, stepSize, numofDOFs, map, x_size, y_size, newIdx);
        if (status != RRT_TRAPPED) {
            finalIdxOut = newIdx;
        }
    } while (status == RRT_ADVANCED);
    return status;
}

static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    auto startTime = std::chrono::steady_clock::now();
    vector<double> startVec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> goalVec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    if (distConfig(startVec, goalVec) < 1e-4) {
        vector<vector<double>> single = {startVec, goalVec};
        generateOutputPlan(single, numofDOFs, plan, planlength);
        return;
    }

    vector<RRTNode> treeA;
    vector<RRTNode> treeB;
    treeA.push_back({startVec, -1});
    treeB.push_back({goalVec, -1});

    bool treeAIsStart = true;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> distAngleRng(0.0, 2.0 * PI);

    const double stepSize = 0.35;
    bool connected = false;
    int connectA = -1;
    int connectB = -1;

    for (int iter = 0; iter < 40000; ++iter) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed > 4500) {
            break;
        }

        vector<double> q_rand(numofDOFs);
        for (int i = 0; i < numofDOFs; ++i) {
            q_rand[i] = distAngleRng(rng);
        }

        int newAIdx = -1;
        RRTExtendStatus statusA = rrtConnectExtend(
            treeA, q_rand, stepSize, numofDOFs, map, x_size, y_size, newAIdx);

        if (statusA != RRT_TRAPPED) {
            int finalBIdx = -1;
            RRTExtendStatus statusB = rrtConnectConnect(
                treeB, treeA[newAIdx].config, stepSize, numofDOFs, map, x_size, y_size, finalBIdx);

            if (statusB == RRT_REACHED) {
                connected = true;
                connectA = newAIdx;
                connectB = finalBIdx;
                break;
            }
        }

        std::swap(treeA, treeB);
        treeAIsStart = !treeAIsStart;
    }

    printf("[STATS] vertices: %zu\n", treeA.size() + treeB.size());
    if (connected) {
        vector<vector<double>> pathA;
        int curr = connectA;
        while (curr != -1) {
            pathA.push_back(treeA[curr].config);
            curr = treeA[curr].parent;
        }
        std::reverse(pathA.begin(), pathA.end());

        vector<vector<double>> pathB;
        curr = connectB;
        while (curr != -1) {
            pathB.push_back(treeB[curr].config);
            curr = treeB[curr].parent;
        }

        vector<vector<double>> fullPath;
        if (treeAIsStart) {
            fullPath = pathA;
            if (!pathB.empty()) {
                fullPath.insert(fullPath.end(), pathB.begin() + 1, pathB.end());
            }
        } else {
            fullPath = pathB;
            std::reverse(fullPath.begin(), fullPath.end());
            if (!pathA.empty()) {
                std::reverse(pathA.begin(), pathA.end());
                fullPath.insert(fullPath.end(), pathA.begin() + 1, pathA.end());
            }
        }

        fullPath.front() = startVec;
        fullPath.back() = goalVec;
        generateOutputPlan(fullPath, numofDOFs, plan, planlength);
    } else {
        planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
    }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

struct RRTStarNode {
    vector<double> config;
    int parent;
    double cost;
    vector<int> children;
};

static void updateRRTStarChildrenCost(vector<RRTStarNode>& tree, int rootIdx) {
    std::queue<int> q;
    for (int child : tree[rootIdx].children) {
        q.push(child);
    }
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        int p = tree[curr].parent;
        tree[curr].cost = tree[p].cost + distConfig(tree[p].config, tree[curr].config);
        for (int child : tree[curr].children) {
            q.push(child);
        }
    }
}

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    auto startTime = std::chrono::steady_clock::now();
    vector<double> startVec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> goalVec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    if (distConfig(startVec, goalVec) < 1e-4) {
        vector<vector<double>> single = {startVec, goalVec};
        generateOutputPlan(single, numofDOFs, plan, planlength);
        return;
    }

    vector<RRTStarNode> tree;
    tree.push_back({startVec, -1, 0.0, {}});

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> distAngleRng(0.0, 2.0 * PI);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);

    const double stepSize = 0.35;
    const double goalBias = 0.05;
    const double gamma = 2.0;

    int bestGoalParent = -1;
    double bestGoalCost = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < 10000; ++iter) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed > 4500) {
            break;
        }
        if (bestGoalParent != -1 && (elapsed > 2000 || iter > 2500)) {
            break;
        }

        vector<double> q_rand(numofDOFs);
        if (dist01(rng) < goalBias) {
            q_rand = goalVec;
        } else {
            for (int i = 0; i < numofDOFs; ++i) {
                q_rand[i] = distAngleRng(rng);
            }
        }

        int nearestIdx = 0;
        double minDist = distConfig(tree[0].config, q_rand);
        for (size_t i = 1; i < tree.size(); ++i) {
            double d = distConfig(tree[i].config, q_rand);
            if (d < minDist) {
                minDist = d;
                nearestIdx = static_cast<int>(i);
            }
        }

        const double stepSize = 0.40;
        int currIdx = nearestIdx;

        for (int s = 0; s < 8; ++s) {
            vector<double> q_new = steerConfig(tree[currIdx].config, q_rand, stepSize);
            if (distConfig(tree[currIdx].config, q_new) < 1e-4) {
                break;
            }
            if (!isValidEdge(tree[currIdx].config, q_new, numofDOFs, map, x_size, y_size)) {
                break;
            }

            double r_n = gamma * pow(log(static_cast<double>(tree.size() + 1)) / (tree.size() + 1), 1.0 / numofDOFs);
            double r_near = std::max(stepSize, std::min(r_n, 1.2));

            vector<pair<double, int>> nearDists;
            for (size_t i = 0; i < tree.size(); ++i) {
                double d = distConfig(tree[i].config, q_new);
                if (d <= r_near) {
                    nearDists.push_back({d, static_cast<int>(i)});
                }
            }
            std::sort(nearDists.begin(), nearDists.end());
            vector<int> nearIndices;
            for (size_t i = 0; i < std::min((size_t)15, nearDists.size()); ++i) {
                nearIndices.push_back(nearDists[i].second);
            }

            int bestParent = currIdx;
            double minCost = tree[currIdx].cost + distConfig(tree[currIdx].config, q_new);

            for (int nearIdx : nearIndices) {
                double c_cand = tree[nearIdx].cost + distConfig(tree[nearIdx].config, q_new);
                if (c_cand < minCost) {
                    if (isValidEdge(tree[nearIdx].config, q_new, numofDOFs, map, x_size, y_size)) {
                        minCost = c_cand;
                        bestParent = nearIdx;
                    }
                }
            }

            int newIdx = static_cast<int>(tree.size());
            tree.push_back({q_new, bestParent, minCost, {}});
            tree[bestParent].children.push_back(newIdx);
            currIdx = newIdx;

            for (int nearIdx : nearIndices) {
                if (nearIdx == bestParent) continue;
                double d_u = distConfig(q_new, tree[nearIdx].config);
                if (minCost + d_u < tree[nearIdx].cost) {
                    if (isValidEdge(q_new, tree[nearIdx].config, numofDOFs, map, x_size, y_size)) {
                        int oldParent = tree[nearIdx].parent;
                        auto& ch = tree[oldParent].children;
                        ch.erase(std::remove(ch.begin(), ch.end(), nearIdx), ch.end());

                        tree[nearIdx].parent = newIdx;
                        tree[newIdx].children.push_back(nearIdx);
                        tree[nearIdx].cost = minCost + d_u;
                        updateRRTStarChildrenCost(tree, nearIdx);
                    }
                }
            }

            double d_goal = distConfig(q_new, goalVec);
            if (d_goal <= 1.2 || d_goal <= r_near) {
                double candGoalCost = tree[newIdx].cost + d_goal;
                if (candGoalCost < bestGoalCost) {
                    if (isValidEdge(q_new, goalVec, numofDOFs, map, x_size, y_size)) {
                        bestGoalCost = candGoalCost;
                        bestGoalParent = newIdx;
                    }
                }
            }

            if (distConfig(q_new, q_rand) < 1e-4) {
                break;
            }
        }
    }

    printf("[STATS] vertices: %zu\n", tree.size());
    if (bestGoalParent != -1) {
        vector<vector<double>> waypoints;
        waypoints.push_back(goalVec);
        int curr = bestGoalParent;
        while (curr != -1) {
            waypoints.push_back(tree[curr].config);
            curr = tree[curr].parent;
        }
        std::reverse(waypoints.begin(), waypoints.end());
        waypoints.front() = startVec;
        waypoints.back() = goalVec;
        generateOutputPlan(waypoints, numofDOFs, plan, planlength);
    } else {
        planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
    }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                SAMPLING HEURISTICS RRT* (NR, LB, CO) IMPLEMENTATION                               //
//                                (Akgun & Stilman 2011: Node Rejection, Local Biasing, Combined)                    //
//                                                                                                                   //
//*******************************************************************************************************************//

enum RRTStarHeuristicMode {
    HEURISTIC_NONE = 0, // Plain (PL)
    HEURISTIC_NR   = 1, // Node Rejection
    HEURISTIC_LB   = 2, // Local Biasing
    HEURISTIC_CO   = 3  // Combined (CO = LB + NR)
};

static vector<double> sampleRRTStar(
    RRTStarHeuristicMode mode,
    int numofDOFs,
    const vector<double>& startVec,
    const vector<double>& goalVec,
    double bestGoalCost,
    const vector<vector<double>>& bestPath,
    double goalBias,
    double pBias,
    double sigma,
    std::mt19937& rng)
{
    std::uniform_real_distribution<double> distAngleRng(0.0, 2.0 * PI);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    std::normal_distribution<double> distNorm(0.0, 1.0);

    bool hasSolution = (bestGoalCost < 1e9 && !bestPath.empty());
    const int maxAttempts = 100;

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        vector<double> q_rand(numofDOFs);

        // 1. Generation Step: Local Biasing (LB) vs Global Exploration
        if (hasSolution && (mode == HEURISTIC_LB || mode == HEURISTIC_CO) && dist01(rng) < pBias) {
            std::uniform_int_distribution<size_t> distNode(0, bestPath.size() - 1);
            size_t chosenIdx = distNode(rng);
            const auto& pt = bestPath[chosenIdx];
            for (int i = 0; i < numofDOFs; ++i) {
                q_rand[i] = wrapAngle(pt[i] + distNorm(rng) * sigma);
            }
        } else {
            if (dist01(rng) < goalBias) {
                q_rand = goalVec;
            } else {
                for (int i = 0; i < numofDOFs; ++i) {
                    q_rand[i] = distAngleRng(rng);
                }
            }
        }

        // 2. Rejection Step: Node Rejection (NR)
        if (hasSolution && (mode == HEURISTIC_NR || mode == HEURISTIC_CO)) {
            double h = distConfig(startVec, q_rand) + distConfig(q_rand, goalVec);
            if (h >= bestGoalCost) {
                continue; // Reject sample!
            }
        }

        return q_rand;
    }

    vector<double> fallback(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) fallback[i] = distAngleRng(rng);
    return fallback;
}

static void plannerRRTStarHeuristic(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength,
    RRTStarHeuristicMode mode)
{
    auto startTime = std::chrono::steady_clock::now();
    vector<double> startVec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> goalVec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    if (distConfig(startVec, goalVec) < 1e-4) {
        vector<vector<double>> single = {startVec, goalVec};
        generateOutputPlan(single, numofDOFs, plan, planlength);
        return;
    }

    vector<RRTStarNode> tree;
    tree.push_back({startVec, -1, 0.0, {}});

    std::mt19937 rng(42);
    const double stepSize = 0.35;
    const double goalBias = 0.05;
    const double pBias = 0.65;    // Probability of local sampling around current path in LB/CO
    const double sigma = 0.30;    // Gaussian dispersion radius
    const double gamma = 2.0;

    int bestGoalParent = -1;
    double bestGoalCost = std::numeric_limits<double>::infinity();
    vector<vector<double>> bestPath;

    auto extractBestPath = [&](int goalParentIdx) {
        vector<vector<double>> path;
        path.push_back(goalVec);
        int curr = goalParentIdx;
        while (curr != -1) {
            path.push_back(tree[curr].config);
            curr = tree[curr].parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    };

    for (int iter = 0; iter < 15000; ++iter) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed > 4500) {
            break;
        }
        if (bestGoalParent != -1 && (elapsed > 3500 || iter > 6000)) {
            break;
        }

        // SAMPLE() with the requested heuristic
        vector<double> q_rand = sampleRRTStar(
            mode, numofDOFs, startVec, goalVec, bestGoalCost, bestPath,
            goalBias, pBias, sigma, rng);

        int nearestIdx = 0;
        double minDist = distConfig(tree[0].config, q_rand);
        for (size_t i = 1; i < tree.size(); ++i) {
            double d = distConfig(tree[i].config, q_rand);
            if (d < minDist) {
                minDist = d;
                nearestIdx = static_cast<int>(i);
            }
        }

        int currIdx = nearestIdx;

        for (int s = 0; s < 8; ++s) {
            vector<double> q_new = steerConfig(tree[currIdx].config, q_rand, stepSize);
            if (distConfig(tree[currIdx].config, q_new) < 1e-4) {
                break;
            }

            // Node Rejection pruning on extension
            if ((mode == HEURISTIC_NR || mode == HEURISTIC_CO) && bestGoalCost < 1e9) {
                double minPotential = tree[currIdx].cost + distConfig(tree[currIdx].config, q_new) + distConfig(q_new, goalVec);
                if (minPotential >= bestGoalCost) {
                    break;
                }
            }

            if (!isValidEdge(tree[currIdx].config, q_new, numofDOFs, map, x_size, y_size)) {
                break;
            }

            double r_n = gamma * pow(log(static_cast<double>(tree.size() + 1)) / (tree.size() + 1), 1.0 / numofDOFs);
            double r_near = std::max(stepSize, std::min(r_n, 1.2));

            vector<pair<double, int>> nearDists;
            for (size_t i = 0; i < tree.size(); ++i) {
                double d = distConfig(tree[i].config, q_new);
                if (d <= r_near) {
                    nearDists.push_back({d, static_cast<int>(i)});
                }
            }
            std::sort(nearDists.begin(), nearDists.end());
            vector<int> nearIndices;
            for (size_t i = 0; i < std::min((size_t)15, nearDists.size()); ++i) {
                nearIndices.push_back(nearDists[i].second);
            }

            int bestParent = currIdx;
            double minCost = tree[currIdx].cost + distConfig(tree[currIdx].config, q_new);

            for (int nearIdx : nearIndices) {
                double c_cand = tree[nearIdx].cost + distConfig(tree[nearIdx].config, q_new);
                if (c_cand < minCost) {
                    if (isValidEdge(tree[nearIdx].config, q_new, numofDOFs, map, x_size, y_size)) {
                        minCost = c_cand;
                        bestParent = nearIdx;
                    }
                }
            }

            int newIdx = static_cast<int>(tree.size());
            tree.push_back({q_new, bestParent, minCost, {}});
            tree[bestParent].children.push_back(newIdx);
            currIdx = newIdx;

            // Rewire neighbors
            for (int nearIdx : nearIndices) {
                if (nearIdx == bestParent) continue;
                double d_u = distConfig(q_new, tree[nearIdx].config);
                if (minCost + d_u < tree[nearIdx].cost) {
                    if (isValidEdge(q_new, tree[nearIdx].config, numofDOFs, map, x_size, y_size)) {
                        int oldParent = tree[nearIdx].parent;
                        auto& ch = tree[oldParent].children;
                        ch.erase(std::remove(ch.begin(), ch.end(), nearIdx), ch.end());

                        tree[nearIdx].parent = newIdx;
                        tree[newIdx].children.push_back(nearIdx);
                        tree[nearIdx].cost = minCost + d_u;
                        updateRRTStarChildrenCost(tree, nearIdx);

                        if (bestGoalParent != -1) {
                            double curGDist = distConfig(tree[bestGoalParent].config, goalVec);
                            double newGCost = tree[bestGoalParent].cost + curGDist;
                            if (newGCost < bestGoalCost) {
                                bestGoalCost = newGCost;
                                bestPath = extractBestPath(bestGoalParent);
                            }
                        }
                    }
                }
            }

            // Check goal connection
            double d_goal = distConfig(q_new, goalVec);
            if (d_goal <= 1.2 || d_goal <= r_near) {
                double candGoalCost = tree[newIdx].cost + d_goal;
                if (candGoalCost < bestGoalCost) {
                    if (isValidEdge(q_new, goalVec, numofDOFs, map, x_size, y_size)) {
                        bestGoalCost = candGoalCost;
                        bestGoalParent = newIdx;
                        bestPath = extractBestPath(bestGoalParent);
                    }
                }
            }

            if (distConfig(q_new, q_rand) < 1e-4) {
                break;
            }
        }
    }

    printf("[STATS] vertices: %zu\n", tree.size());
    if (bestGoalParent != -1) {
        vector<vector<double>> waypoints = extractBestPath(bestGoalParent);
        generateOutputPlan(waypoints, numofDOFs, plan, planlength);
    } else {
        planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
    }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    auto startTime = std::chrono::steady_clock::now();
    vector<double> startVec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    vector<double> goalVec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    if (distConfig(startVec, goalVec) < 1e-4) {
        printf("[STATS] vertices: 2\n");
        vector<vector<double>> single = {startVec, goalVec};
        generateOutputPlan(single, numofDOFs, plan, planlength);
        return;
    }

    if (isValidEdge(startVec, goalVec, numofDOFs, map, x_size, y_size)) {
        printf("[STATS] vertices: 2\n");
        vector<vector<double>> direct = {startVec, goalVec};
        generateOutputPlan(direct, numofDOFs, plan, planlength);
        return;
    }

    vector<vector<double>> nodes;
    nodes.push_back(startVec); // index 0
    nodes.push_back(goalVec);  // index 1

    vector<vector<pair<int, double>>> adj(2);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> distAngleRng(0.0, 2.0 * PI);

    const int kNeighbors = 12;
    const double maxConnectionDist = 1.4;

    auto addNodeAndConnect = [&](const vector<double>& q) -> int {
        int u = static_cast<int>(nodes.size());
        nodes.push_back(q);
        adj.emplace_back();

        vector<pair<double, int>> dists;
        dists.reserve(u);
        for (int v = 0; v < u; ++v) {
            double d = distConfig(q, nodes[v]);
            if (d <= maxConnectionDist) {
                dists.push_back({d, v});
            }
        }
        std::sort(dists.begin(), dists.end());
        int count = 0;
        for (const auto& pr : dists) {
            if (count >= kNeighbors) break;
            int v = pr.second;
            double d = pr.first;
            if (isValidEdge(nodes[u], nodes[v], numofDOFs, map, x_size, y_size)) {
                adj[u].push_back({v, d});
                adj[v].push_back({u, d});
                count++;
            }
        }
        return u;
    };

    // Attempt direct start-goal connection if within range
    if (distConfig(startVec, goalVec) <= maxConnectionDist) {
        if (isValidEdge(startVec, goalVec, numofDOFs, map, x_size, y_size)) {
            double d = distConfig(startVec, goalVec);
            adj[0].push_back({1, d});
            adj[1].push_back({0, d});
        }
    }

    bool pathFound = false;
    vector<int> solvedPathIndices;

    int batchSize = 400;
    while (!pathFound) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed > 4500) {
            break;
        }

        int sampled = 0;
        int attempts = 0;
        while (sampled < batchSize && attempts < batchSize * 50) {
            attempts++;
            vector<double> cand(numofDOFs);
            for (int i = 0; i < numofDOFs; ++i) {
                cand[i] = distAngleRng(rng);
            }
            if (IsValidArmConfiguration(cand.data(), numofDOFs, map, x_size, y_size)) {
                addNodeAndConnect(cand);
                sampled++;
            }
        }

        // A* search from 0 to 1
        int nNodes = static_cast<int>(nodes.size());
        vector<double> gScore(nNodes, std::numeric_limits<double>::infinity());
        vector<int> cameFrom(nNodes, -1);
        gScore[0] = 0.0;

        using PQElem = pair<double, int>;
        std::priority_queue<PQElem, vector<PQElem>, std::greater<PQElem>> pq;
        pq.push({distConfig(nodes[0], nodes[1]), 0});

        while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();
            double f = top.first;
            int curr = top.second;

            if (curr == 1) {
                pathFound = true;
                break;
            }

            double curG = gScore[curr];
            if (f > curG + distConfig(nodes[curr], nodes[1]) + 1e-6) {
                continue;
            }

            for (const auto& edge : adj[curr]) {
                int next = edge.first;
                double w = edge.second;
                double tentativeG = curG + w;
                if (tentativeG < gScore[next]) {
                    gScore[next] = tentativeG;
                    cameFrom[next] = curr;
                    double h = distConfig(nodes[next], nodes[1]);
                    pq.push({tentativeG + h, next});
                }
            }
        }

        if (pathFound) {
            int curr = 1;
            while (curr != -1) {
                solvedPathIndices.push_back(curr);
                curr = cameFrom[curr];
            }
            std::reverse(solvedPathIndices.begin(), solvedPathIndices.end());
            break;
        }

        batchSize = 150;
    }

    printf("[STATS] vertices: %zu\n", nodes.size());
    if (pathFound) {
        vector<vector<double>> waypoints;
        for (int idx : solvedPathIndices) {
            waypoints.push_back(nodes[idx]);
        }
        waypoints.front() = startVec;
        waypoints.back() = goalVec;
        generateOutputPlan(waypoints, numofDOFs, plan, planlength);
    } else {
        planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
    }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    // g++ builds record a path relative to the process cwd (often build/).
    // visualization is run from the code directory, so store an absolute path.
#ifndef _WIN32
    char resolvedMapPath[PATH_MAX];
    if (realpath(mapFilePath.c_str(), resolvedMapPath) != nullptr) {
        mapFilePath = resolvedMapPath;
    }
#endif
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
	tie(map, x_size, y_size) = loadMap(mapFilePath);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);

    std::string outputDir = OUTPUT_DIR;
	string outputFile = outputDir + "/" + argv[6];
	std::cout << "Writing solution to: " << outputFile << std::endl;

#ifndef _WIN32
    if (argc > 7) {
        setenv("RRT_GOAL_BIAS", argv[7], 1);
    }
#endif

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR_NR)
    {
        plannerRRTStarHeuristic(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, HEURISTIC_NR);
    }
    else if (whichPlanner == RRTSTAR_LB)
    {
        plannerRRTStarHeuristic(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, HEURISTIC_LB);
    }
    else if (whichPlanner == RRTSTAR_CO)
    {
        plannerRRTStarHeuristic(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, HEURISTIC_CO);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << mapFilePath << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
