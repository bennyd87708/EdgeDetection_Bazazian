// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include <E57SimpleReader.h>
#include <E57SimpleWriter.h>

using namespace std;

int main()
{
    e57::Writer eWriter("../../../../data/test.e57");
    e57::Data3D	scanHeader;
    scanHeader.guid = "{D3817EC3-A3DD-4a81-9EF5-2FFD0EC91D5A}";
    int totalPoints = 20;
    scanHeader.pointCount = totalPoints;
    scanHeader.pointFields.cartesianXField = true;
    scanHeader.pointFields.cartesianYField = true;
    scanHeader.pointFields.cartesianZField = true;
    int scanIndex = eWriter.NewData3D(scanHeader);
    e57::Data3DPointsData_t<double> pointsData;
    e57::CompressedVectorWriter dataWriter = eWriter.SetUpData3DPointsData(scanIndex, totalPoints, pointsData);
    for (std::size_t i = 0; i < totalPoints; i++) {
        pointsData.cartesianX[i] = 1024 * rand() / (RAND_MAX + 1.0f);
        pointsData.cartesianY[i] = 1024 * rand() / (RAND_MAX + 1.0f);
        pointsData.cartesianZ[i] = 1024 * rand() / (RAND_MAX + 1.0f);
        pointsData.colorRed[i] = 255 * rand();
        pointsData.colorGreen[i] = 255 * rand();
        pointsData.colorBlue[i] = 255 * rand();
    }

    // write the mesh data
    dataWriter.write(totalPoints);
    dataWriter.close();
    eWriter.Close();

    /*
    e57::E57Root e57FileInfo{ };
    e57::Reader  e57FileReader{ "../../../../data/test.e57" };

    // check if the file is opened
    e57FileReader.IsOpen();
    // read E57 root to explore the tree
    e57FileReader.GetE57Root(e57FileInfo);

    int64_t data3DCount = e57FileReader.GetData3DCount();
    */
	cin.get();
	return 0;
}
