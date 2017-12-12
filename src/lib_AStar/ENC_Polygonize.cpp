#include "ENC_Polygonize.h"

ENC_Polygonize::ENC_Polygonize()
{
    landZ = 5;
    MOOS_PATH = "/home/sreed/moos-ivp/moos-ivp-reed/";
    outfile_path = MOOS_PATH+"/src/ENCs/Grid/raster.shp";
    tiff_path = MOOS_PATH+"/src/ENCs/Grid/new.tiff";
    geod = Geodesy();
    minDepth = 1; // in meters
    closed = false;
    buildSHP();
}

void ENC_Polygonize::buildSHP()
{
    GDALAllRegister();
    // Build the datasets
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    OGRFieldDefn oField_depth( "Depth", OFTReal);

    // Build the datasource and layer for the new shapefile
    ds = poDriver->Create(outfile_path.c_str(), 0, 0, 0, GDT_Unknown, NULL );
    if( ds == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }

    // Create the layers
    layer = ds->CreateLayer( "poly", NULL, wkbPolygon25D, NULL );
    if( layer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }
    // Create the field
    if(layer->CreateField( &oField_depth ) != OGRERR_NONE )
    {
        printf( "Creating Depth field failed.\n" );
        exit( 1 );
    }
    feat_def = layer->GetLayerDefn();
}

void ENC_Polygonize::polygonize()
{
    GDALAllRegister();
    GDALDataset  *ds_raster;
    GDALRasterBand  *rasterBand;
    char **papszOptions = NULL;//"8CONNECTED";

    // Make a binary grid of the ENC grid based on the minimum depth
    string binaryFilepath= makeBinaryGrid();

    // Get the band for the binary grid
    ds_raster = (GDALDataset *) GDALOpen( binaryFilepath.c_str(), GA_ReadOnly );
    rasterBand = ds_raster->GetRasterBand( 1 );

    cout << "Starting to Polygonize" << endl;
    if (GDALPolygonize(rasterBand, NULL, layer, 0, papszOptions,NULL, NULL) != CE_None)
    {
        cout << "IT FAILED! :(" << endl;
        GDALClose(ds_raster);
        exit(1);
    }
    GDALClose(ds_raster);
}

string ENC_Polygonize::makeBinaryGrid()
{
    int nXSize, nYSize;
    vector<float> ENC_data;
    vector<double> geoTransform;
    geoTransform.resize(6);
    string binaryGrid_filename ="binary.tiff";

    // Store the data from the tiff
    getRasterData(tiff_path, nXSize, nYSize, ENC_data, geoTransform);

    vector<int> binaryGrid;
    cout << "minDepth: " << minDepth << endl;
    // Loop through the raster and only store the areas that have a depth less than the minimum and are not land
    for (int i=0; i<ENC_data.size(); i++)
    {
        if (ENC_data[i]==-10)
            binaryGrid.push_back(-1);
        else if (ENC_data[i]<=landZ)
        {
            binaryGrid.push_back(0);
        }
        else if ((ENC_data[i]<=minDepth)&&(ENC_data[i]>landZ))
            binaryGrid.push_back(1);
        else
            binaryGrid.push_back(2);
    }

    // Write a new file
    writeRasterData(binaryGrid_filename, nXSize, nYSize, binaryGrid, geoTransform);

    return MOOS_PATH+"src/ENCs/Grid/" +binaryGrid_filename;
}

void ENC_Polygonize::getRasterData(string filename, int &nXSize, int &nYSize, vector<float> &RasterData, vector<double> &adfGeoTransform)
{
    GDALDataset  *poDataset;
    GDALRasterBand *poRasterBand;
    //string full_Filename = MOOS_PATH+"src/ENCs/Grid/" +filename;

    GDALAllRegister();
    poDataset = (GDALDataset *) GDALOpen( filename.c_str(), GA_ReadOnly );
    poDataset->GetGeoTransform(adfGeoTransform.data());

    poRasterBand = poDataset -> GetRasterBand(1);

    nXSize = poRasterBand->GetXSize(); // width
    nYSize = poRasterBand->GetYSize(); // height

    // Resize the vector to fit the raster dataset
    RasterData.resize(nXSize*nYSize);

    // Read the raster into a 1D row-major vector where it is organized in left to right,top to bottom pixel order
    if( poRasterBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, RasterData.data(), nXSize, nYSize, GDT_Float32, 0, 0 ) != CE_None )
    {
        printf( "Failed to access raster data.\n" );
        GDALClose(poDataset);
        exit( 1 );
    }

    GDALClose(poDataset);
}

void ENC_Polygonize::writeRasterData(string filename, int nXSize, int nYSize, vector<int> &RasterData, vector<double> &adfGeoTransform)
{
    GDALDataset *poDataset;
    GDALRasterBand *poBand;
    OGRSpatialReference oSRS;
    GDALDriver *poDriver;
    char *pszSRS_WKT = NULL;
    char **papszOptions = NULL;
    const char *pszFormat = "GTiff";

    string full_Filename = MOOS_PATH+"src/ENCs/Grid/" +filename;

    //cout << "File name: " << full_Filename << endl;

    // Open file for writing
    poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    poDataset = poDriver->Create( full_Filename.c_str(), nXSize, nYSize, 1, GDT_Int32, papszOptions );

    // Set geo transform
    poDataset->SetGeoTransform( adfGeoTransform.data() );

    // Set Coordinate system
    oSRS = geod.getUTM();
    oSRS.exportToWkt( &pszSRS_WKT );
    poDataset->SetProjection( pszSRS_WKT );
    CPLFree( pszSRS_WKT );

    // Write the file
    poBand = poDataset->GetRasterBand(1);
    if (poBand->RasterIO( GF_Write, 0, 0, nXSize, nYSize,
                      RasterData.data(), nXSize, nYSize, GDT_Int32, 0, 0 ) != CE_None )
    {
        printf( "Failed to write raster data.\n" );
        GDALClose(poDataset);
        exit( 1 );
    }

    // Once we're done, close properly the dataset
    GDALClose( (GDALDatasetH) poDataset );
}

void ENC_Polygonize::runWithGrid(string ENC_Name, double grid_size, double buffer_size, double MHW_offset, bool simpleGrid)
{
    cout << "Gridding..." << endl;
    // Build the gridded ENC
    Grid_Interp grid = Grid_Interp(MOOS_PATH, MOOS_PATH+"src/ENCs/"+ENC_Name+"/"+ENC_Name+".000", grid_size, buffer_size, MHW_offset, geod, simpleGrid);
    grid.Run(false);
    landZ = grid.getLandZ()/100;

    polygonize();
}
