#include "gdal_priv.h"
#include "cpl_conv.h" // for CPLMalloc()

int implicit() {
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
    int   CHUNK_SIZE = 4;
    int   GRID_SIZE = 1;
    double PIXEL_WIDTH = adfGeoTransform[1];
    int   X_ORIGIN = adfGeoTransform[0];
    int   Y_ORIGIN = adfGeoTransform[3]; // Starting from top left corner


};