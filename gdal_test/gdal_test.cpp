#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <array>
#include <cstdlib>
#include <math.h>
#include <fstream>
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()
#include "half.hpp"


GDALRasterBand* poBand;
// Global Variables
int   CHUNK_SIZE = 64;
int   GRID_SIZE = 4;
double PIXEL_WIDTH;
int   X_ORIGIN, Y_ORIGIN;
int   nodes = 0;
std::ofstream Encoded("encoded.txt");

using namespace Eigen;

class Chunk {
private:

public:
    int size, xOffset, yOffset, increment;
    int** chunk;
    Vector3f fit;
    std::vector<float> altVector;
    float info;

    // Constructor
    Chunk(int size, int xOffset, int yOffset, int increment) {
        Chunk* e = this;
        e->size = size;
        e->xOffset = xOffset;
        e->yOffset = yOffset;
        e->increment = increment;

        chunk = getChunk(poBand);
        altVector = altitudeVector();
        fit = fitVector();
        half i(fit[0]), j(fit[1]), k(fit[2]);

        info = information(i, j, k);

        subchunk(i, j, k);
        
    }


    std::array<double, 4> chunkLocation(double X_ORIGIN, double Y_ORIGIN, double PIXEL_WIDTH) {
        double xMin = X_ORIGIN - (xOffset * PIXEL_WIDTH);
        double xMax = X_ORIGIN - ((xOffset + size) * PIXEL_WIDTH);
        double yMin = Y_ORIGIN - (yOffset * PIXEL_WIDTH);
        double yMax = Y_ORIGIN - ((yOffset + size) * PIXEL_WIDTH);

        std::array<double, 4> arr = { xMin, xMax, yMin, yMax };
        return arr;
    }


    int** getChunk(GDALRasterBand* poBand) {
        int** block;
        block = (int**)CPLMalloc(sizeof(int*) * size*size);

        for (int row = 0; row < size; row++) {
            block[row] = (int*)CPLMalloc(sizeof(int) * size); // Allocating memory for next row  
            poBand->RasterIO(GF_Read,
                xOffset, yOffset + row, size, 1,
                block[row], size, 1, GDT_Int32,
                0, 0);
        }

        return block;
    }

    Eigen::Vector3f fitVector() {
        int size = this->size;
        int sq = size * size;
        float* fVptr = &altVector[0];              // Necessary for Eigen::Map

        Eigen::Map<Eigen::VectorXf> b(fVptr, size * size);


        Eigen::VectorXi v(sq), a1(sq), a2(sq), a3(sq);

        v = Eigen::VectorXi::LinSpaced(sq, 0, sq - 1);
        a1 = v.unaryExpr([size](const int x) { return x % size; });  // Hacky mod(size) to get every x
        a2 = v / size;                                               // Get every y by dividing by 8 (integer typecast auto floor)
        a3.setConstant(1);

        Eigen::MatrixXi m(sq, 3);
        m << a1, a2, a3;
        Eigen::MatrixXf f = m.cast<float>();

        Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(f);  // Convienent Eigen function to solve for x-bar vector. Saves difficult matrix algebra with LU algorithm
        Eigen::Vector3f x = dec.solve(b);

        return x;
    }


    std::vector<float> altitudeVector() {
        std::vector<float> floatVector;

        for (int q = 0; q < size; q++) {
            for (int t = 0; t < size; t++) {
                floatVector.push_back(chunk[q][t]);  // Turns the 2d array into a 1d std::vector so it can be mapped to b vector
            }
        }

        return floatVector;
    }

    float information(half i, half j, half k) {
        int size = this->size;
        float info = 0;

        for (int row = 0; row < size; row++) {
            for (int elem = 0; elem < size; elem++) {
                float z = (elem * i) + (row * j) + k;  // In form ax+by+z for vector
                float dif = altVector[(row*size)+elem] - z;  // Subtracts the correct value from each
                float score = std::abs(dif / 30);  // Assuming sigma = 30 (arbitrary)
                float P_ak = 1 / std::pow(4.0, score);
                float pointInfo = -(std::log2(P_ak));

                info += pointInfo;
            }
        }
        float adjusted = (info * (increment * increment) / (size * size));

        return adjusted;
    }

    void subchunk(half i, half j, half k) {
        if (info > 0.3 && size >= 4) {
            int new_size = size / 2;
            Chunk child1 = Chunk(new_size, xOffset, yOffset, increment);
            Chunk child2 = Chunk(new_size, xOffset + new_size, yOffset, increment);
            Chunk child3 = Chunk(new_size, xOffset, yOffset + new_size, increment);
            Chunk child4 = Chunk(new_size, xOffset + new_size, yOffset + new_size, increment);
        }
        else {
            nodes++;
    
 
            Encoded << i << " " << j << " " << k << " " << size << "\n";
        }
    }


};


int main() {
    // Initialize GDAL with file
    GDALDataset* poDataset;
    GDALAllRegister();
    const char* pszFilename = "C:\\Users\\jacki\\PycharmProjects\\pythonProject2\\tifs\\kansas_n32_w093_1arc_v1.tif";
    poDataset = (GDALDataset*)GDALOpen(pszFilename, GA_ReadOnly);

    poBand = poDataset->GetRasterBand(1);

    static int nXSize = poBand->GetXSize();
    static int nYSize = poBand->GetYSize();
    double adfGeoTransform[6];
    poDataset->GetGeoTransform(adfGeoTransform);

    // Global Variables
    PIXEL_WIDTH = adfGeoTransform[1];
    X_ORIGIN = adfGeoTransform[0];
    Y_ORIGIN = adfGeoTransform[3]; // Starting from top left corner


    if (CHUNK_SIZE * GRID_SIZE > (nXSize | nYSize)) {
        std::cerr << "Grid exceeds original raster size";
        exit(1);
    }


    for (int gridRow = 0; gridRow < GRID_SIZE; gridRow++) {
        for (int gridElem = 0; gridElem < GRID_SIZE; gridElem++) {
            int workingOffsetX = gridElem * CHUNK_SIZE;
            int workingOffsetY = gridRow * CHUNK_SIZE;

            Chunk working = Chunk(CHUNK_SIZE, workingOffsetX, workingOffsetY, 1);

            

            CPLFree(working.chunk);  // Frees the memory malloced in Chunk.getChunk method for returned array
        }
    }

    std::cout << nodes;
}