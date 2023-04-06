// main.cpp : Defines the entry point for the application.
//

#include "main.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <eigen/dense>
#include <eigen/eigenvalues>
#include <iomanip>
#include <ctime>
#include <E57SimpleReader.h>
#include <E57SimpleWriter.h>

using namespace e57;
using namespace Eigen;
using namespace std;
using namespace pcl;

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
        pointsData.cartesianZ[i] = randDouble(3);
        pointsData.colorRed[i] = int(randDouble(255));
        pointsData.colorGreen[i] = int(randDouble(255));
        pointsData.colorBlue[i] = int(randDouble(255));

        if (verbose)
        printf("Point #%-6dLocation: %-13f%-13f%-15fColor: %-5d%-5d%-5d\n", i,
            pointsData.cartesianX[i], pointsData.cartesianY[i], pointsData.cartesianZ[i],
            pointsData.colorRed[i], pointsData.colorGreen[i], pointsData.colorBlue[i]);
    }

    for (int i = POINTS / 2; i < POINTS; i++) {
        pointsData.cartesianX[i] = randDouble(3);
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

void markEdges(PointCloud<PointXYZRGBA>::Ptr cloud) {
    cout << "Marking edges\n";

    KdTreeFLANN<PointXYZRGBA> kdtree;
    kdtree.setInputCloud(cloud);

    for (int i = 0; i < POINTS; i++) {
        PointXYZRGBA searchPoint = (*cloud)[i];
        int K = 50;
        vector<int> indices(K);
        vector<float> distances(K);
        if (kdtree.nearestKSearch(searchPoint, K, indices, distances) <= 0) {
            (*cloud)[i].r = 0;
            (*cloud)[i].g = 0;
            (*cloud)[i].b = 0;
            continue;
        }

        MatrixXd mat(K, 3);
        for (int j = 0; j < K; j++) {
            mat(j, 0) = (*cloud)[indices[j]].x;
            mat(j, 1) = (*cloud)[indices[j]].y;
            mat(j, 2) = (*cloud)[indices[j]].z;
        }
        MatrixXd centered = mat.rowwise() - mat.colwise().mean();
        MatrixXd cov = (centered.adjoint() * centered) / double(mat.rows() - 1);

        EigenSolver<Matrix3d> es(cov, false);
        Vector3cd D = es.eigenvalues();
        Vector3d R = D.real();

        double var = min(R[0], min(R[1], R[2])) / (R[0] + R[1] + R[2]);

        if (var < 0.005) {
            (*cloud)[i].r = 255;
            (*cloud)[i].g = 255;
            (*cloud)[i].b = 255;
        }
        else {
            (*cloud)[i].r = 255;
            (*cloud)[i].g = 0;
            (*cloud)[i].b = 0;
        }
    }
}

PointCloud<PointXYZRGBA>::Ptr convertToPCLCloud(Data3DPointsDouble& data) {
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>);
    cloud->width = POINTS;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        (*cloud)[i].x = data.cartesianX[i];
        (*cloud)[i].y = data.cartesianY[i];
        (*cloud)[i].z = data.cartesianZ[i];
        (*cloud)[i].r = data.colorRed[i];
        (*cloud)[i].g = data.colorGreen[i];
        (*cloud)[i].b = data.colorBlue[i];
    }
    return cloud;
}

void convertToData(Data3DPointsDouble& data, PointCloud<PointXYZRGBA>::Ptr cloud) {
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        data.cartesianX[i] = (*cloud)[i].x;
        data.cartesianY[i] = (*cloud)[i].y;
        data.cartesianZ[i] = (*cloud)[i].z;
        data.colorRed[i]   = (*cloud)[i].r;
        data.colorGreen[i] = (*cloud)[i].g;
        data.colorBlue[i]  = (*cloud)[i].b;
    }
}

void visualizePCLCloud(PointCloud<PointXYZRGBA>::Ptr cloud) {
    visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}

int main() {
    writeTestFile();

    Data3D header;
    Data3DPointsDouble data = readFile("test.e57", header);
    PointCloud<PointXYZRGBA>::Ptr cloud = convertToPCLCloud(data);

    markEdges(cloud);

    convertToData(data, cloud);
    writeFile("test.e57", header, data);
    readFile("test.e57", header);

    visualizePCLCloud(cloud);

    cout << "Completed.";
    cin.get();
    return 0;
}