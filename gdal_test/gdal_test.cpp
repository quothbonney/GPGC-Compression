#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <array>
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()


GDALRasterBand* poBand;
// Global Variables
int   CHUNK_SIZE = 4;
int   GRID_SIZE = 1;
double PIXEL_WIDTH;
int   X_ORIGIN, Y_ORIGIN;
using namespace Eigen;

class Chunk {
private:

public:
    int size, xOffset, yOffset, increment;
    int** chunk;
    Vector3f fit;
    std::vector<float> altVector, expected;

    // Constructor
    Chunk(int size, int xOffset, int yOffset, int increment) {
        Chunk* e = this;
        e->size = size;
        e->xOffset = xOffset;
        e->yOffset = yOffset;
        e->increment = increment;

        chunk = getChunk(poBand);
        altVector = altitudeVector(chunk, size);
        fit = fitVector();

        expected = expectedVector();
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
        block = (int**)CPLMalloc(sizeof(int*) * size);

        for (int row = xOffset; row < size; row++) {
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


    std::vector<float> altitudeVector(int** block, int size) {
        std::vector<float> floatVector;

        for (int q = 0; q < size; q++) {
            for (int t = 0; t < size; t++) {
                floatVector.push_back(block[q][t]);  // Turns the 2d array into a 1d std::vector so it can be mapped to b vector
            }
        }

        return floatVector;
    }

    std::vector<float> expectedVector() {
        int size = this->size;

        std::vector<float> expect;
        for (int row = 0; row < size; row++) {
            for (int elem = 0; elem < size; elem++) {
                float z = (elem * fit[0]) + (row * fit[1]) + fit[2];
                expect.push_back(z);
            }
        }
        return expect;
    }


};


int main() {
    // Initialize GDAL with file
    GDALDataset* poDataset;
    GDALAllRegister();
    const char* pszFilename = "C:\\Users\\jacki\\source\\repos\\gdal_test\\gdal_test\\x64\\src\\n00_e010_1arc_v3.tif";
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



    Chunk test = Chunk(CHUNK_SIZE, 0, 0, 1);

    auto alts = test.altVector;
    auto expts = test.expected;

    for (int i = 0; i < CHUNK_SIZE * CHUNK_SIZE; i++) {
        float dif = alts[i] - expts[i];
        std::cout << dif << std::endl;
    }
}