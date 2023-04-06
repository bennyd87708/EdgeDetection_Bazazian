// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include <eigen/dense>
#include <eigen/eigenvalues>
#include <iomanip>
#include <ctime>
#include <E57SimpleReader.h>
#include <E57SimpleWriter.h>

using namespace e57;
using namespace Eigen;
using namespace std;

static int POINTS = 50000;

double randDouble(int max) {
    return max * rand() / (RAND_MAX + 1.0f);
}

void setUsingColoredCartesianPoints(Data3D& header)
{
    header.pointFields.cartesianXField = true;
    header.pointFields.cartesianYField = true;
    header.pointFields.cartesianZField = true;

    header.pointFields.colorRedField = true;
    header.pointFields.colorGreenField = true;
    header.pointFields.colorBlueField = true;

    header.colorLimits.colorRedMaximum = 255;
    header.colorLimits.colorGreenMaximum = 255;
    header.colorLimits.colorBlueMaximum = 255;
}

Data3DPointsDouble readFile(string filename, Data3D& data3DHeader, bool verbose = false) {
    cout << "Reading data from " << filename << "\n";

    string location = "../../../../data/" + filename;
    Reader reader(location, {});

    E57Root header;
    reader.GetE57Root(header);
    int blockCount = reader.GetData3DCount();

    reader.ReadData3D(0, data3DHeader);
    int pointCount = data3DHeader.pointCount;

    Data3DPointsDouble pointsData(data3DHeader);
    CompressedVectorReader vectorReader = reader.SetUpData3DPointsData(0, pointCount, pointsData);

    vectorReader.read();
    vectorReader.close();

    reader.Close();

    if (verbose) {
        cout << "GUID: " << header.guid << "\n";
        cout << "Block Count: " << blockCount << "\n";
        cout << "Block GUID: " << data3DHeader.guid << "\n";
        cout << "Point Count: " << pointCount << "\n";
        for (int i = 0; i < pointCount; i++) {
            printf("Point #%-6dLocation: %-13f%-13f%-15fColor: %-5d%-5d%-5d\n", i,
                pointsData.cartesianX[i], pointsData.cartesianY[i], pointsData.cartesianZ[i],
                pointsData.colorRed[i], pointsData.colorGreen[i], pointsData.colorBlue[i]);
        }
    }

    return pointsData;
}

void writeFile(string filename, Data3D &header, Data3DPointsDouble &data) {
    cout << "Writing data to " << filename << "\n";

    WriterOptions options;
    options.guid = "Test File GUID";
    string location = "../../../../data/" + filename;
    Writer writer(location, options);

    writer.WriteData3DData(header, data);
    writer.Close();
}

void writeTestFile(bool verbose = false) {
    srand(time(NULL));
    cout << "Generating data\n";
    Data3D header;
    setUsingColoredCartesianPoints(header);
    header.guid = "Test File Scan Header GUID";
    header.description = "libE57Format test write file";
    header.pointCount = POINTS;
    header.pointFields.pointRangeMinimum = 0.0;
    header.pointFields.pointRangeMaximum = 1024.0;

    Data3DPointsDouble pointsData(header);

    for (int i = 0; i < POINTS/2; i++) {
        pointsData.cartesianX[i] = randDouble(1024);
        pointsData.cartesianY[i] = randDouble(1024);
        pointsData.cartesianZ[i] = randDouble(5);
        pointsData.colorRed[i] = int(randDouble(255));
        pointsData.colorGreen[i] = int(randDouble(255));
        pointsData.colorBlue[i] = int(randDouble(255));

        if (verbose)
        printf("Point #%-6dLocation: %-13f%-13f%-15fColor: %-5d%-5d%-5d\n", i,
            pointsData.cartesianX[i], pointsData.cartesianY[i], pointsData.cartesianZ[i],
            pointsData.colorRed[i], pointsData.colorGreen[i], pointsData.colorBlue[i]);
    }

    for (int i = POINTS / 2; i < POINTS; i++) {
        pointsData.cartesianX[i] = randDouble(5);
        pointsData.cartesianY[i] = randDouble(1024);
        pointsData.cartesianZ[i] = randDouble(1024);
        pointsData.colorRed[i] = int(randDouble(255));
        pointsData.colorGreen[i] = int(randDouble(255));
        pointsData.colorBlue[i] = int(randDouble(255));

        if (verbose)
        printf("Point #%-6dLocation: %-13f%-13f%-15fColor: %-5d%-5d%-5d\n", i,
            pointsData.cartesianX[i], pointsData.cartesianY[i], pointsData.cartesianZ[i],
            pointsData.colorRed[i], pointsData.colorGreen[i], pointsData.colorBlue[i]);
    }

    writeFile("test.e57", header, pointsData);
}

void markEdges(Data3DPointsDouble &data) {
    cout << "Marking Edges\n";

    MatrixXd mat(POINTS, 3);
    for (int i = 0; i < POINTS; i++) {
        mat(i, 0) = data.cartesianX[i];
        mat(i, 1) = data.cartesianY[i];
        mat(i, 2) = data.cartesianZ[i];
    }
    Vector3d mean = mat.colwise().mean();

    VectorXd variations(POINTS);
    for (int i = 0; i < POINTS; i++) {
        double x = mat(i, 0) - mean[0];
        double y = mat(i, 1) - mean[1];
        double z = mat(i, 2) - mean[2];
        MatrixXd cov{
            {x * x, x * y, x * z},
            {y * x, y * y, y * z},
            {z * x, z * y, z * z}
        };

        EigenSolver<Matrix3d> es(cov);
        Matrix3d D = es.pseudoEigenvalueMatrix();
        double lambda_0 = D(0, 0);
        double lambda_1 = D(1, 1);
        double lambda_2 = D(2, 2);

        variations[i] = lambda_0 / (lambda_0 + lambda_1 + lambda_2);

        if (variations[i] > 0.99) {
            data.colorRed[i] = 255;
            data.colorBlue[i] = 255;
            data.colorGreen[i] = 255;
        }
        else {
            data.colorRed[i] = 1;
            data.colorBlue[i] = 1;
            data.colorGreen[i] = 1;
        }
    }
}

int main() {
    writeTestFile();

    Data3D header;
    Data3DPointsDouble data = readFile("test.e57", header);

    markEdges(data);

    writeFile("test.e57", header, data);
    readFile("test.e57", header);

    cin.get();
    return 0;
}