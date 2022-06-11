#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <array>
#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()



class Chunk {
private:
    int** getChunk(GDALRasterBand* poBand) {
        int** block;
        block = (int**)CPLMalloc(sizeof(int*) * size);

        for (int row = xOffset; row < size; row++) {
            block[row] = (int*)CPLMalloc(sizeof(int) * size); // Allocating memory for next row  
            poBand->RasterIO(GF_Read, 
                xOffset, yOffset + row, size, 1,
                block[row], size, 1, GDT_Int32,
                0, 0);
            int* intPtr = block[row];
        }

        return block;
    }

public:
    int size;
    int xOffset;
    int yOffset;
    int increment;
    int** chunk;

    // Constructor
    Chunk(int chunkSize, int chunkOffsetX, int chunkOffsetY, int chunkInc, GDALRasterBand* band) {
        size = chunkSize;
        xOffset = chunkOffsetX;
        yOffset = chunkOffsetY;

        chunk = getChunk(band);
    }



    std::array<double, 4> chunkLocation(double X_ORIGIN, double Y_ORIGIN, double PIXEL_WIDTH) {
        double xMin = X_ORIGIN - (xOffset * PIXEL_WIDTH);
        double xMax = X_ORIGIN - ((xOffset + size) * PIXEL_WIDTH);
        double yMin = Y_ORIGIN - (yOffset * PIXEL_WIDTH);
        double yMax = Y_ORIGIN - ((yOffset + size) * PIXEL_WIDTH);

        std::array<double, 4> arr = { xMin, xMax, yMin, yMax };
        return arr;
    }



    void chunkVector() {
        struct pointVals {
            int xIndex;
            int yIndex;
            int zVal;
        };
    }


    
};


int main() {
    // Initialize GDAL with file
    GDALDataset* poDataset;
    GDALAllRegister();
    const char* pszFilename = "C:\\Users\\jacki\\source\\repos\\gdal_test\\gdal_test\\x64\\src\\n00_e010_1arc_v3.tif";
    poDataset = (GDALDataset*)GDALOpen(pszFilename, GA_ReadOnly);

    auto poBand = poDataset->GetRasterBand(1);

    static int nXSize = poBand->GetXSize();
    static int nYSize = poBand->GetYSize();
    double adfGeoTransform[6];
    poDataset->GetGeoTransform(adfGeoTransform);

    // Global Variables
    int   CHUNK_SIZE = 16;
    int   GRID_SIZE = 1;
    double PIXEL_WIDTH = adfGeoTransform[1];
    int   X_ORIGIN = adfGeoTransform[0];
    int   Y_ORIGIN = adfGeoTransform[3]; // Starting from top left corner



    if (CHUNK_SIZE * GRID_SIZE > (nXSize | nYSize)) {
        std::cerr << "Grid exceeds original raster size";
        exit(1);
    }


    Chunk test = Chunk(CHUNK_SIZE, 0, 0, 1, poBand);
    std::array<double, 4> arr = test.chunkLocation(X_ORIGIN, Y_ORIGIN, PIXEL_WIDTH);

    for (int i = 0; i < CHUNK_SIZE; ++i) {
        for (int j = 0; j < CHUNK_SIZE; ++j) {
            std::cout << test.chunk[i][j] << " ";
        }
        std::cout << "\n";
    }
}