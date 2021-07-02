#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <list>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include<fstream>

using namespace cv;
using std::cout;
using std::cerr;
using std::endl;
using namespace std;

// Stores frames along with their timestamps
struct Frame
{
    int64 timestamp;
    Mat frame;
};

int main()
{
    char DepthimgName[500];
    char ColorimgName[500];

    //! [Open streams]
    // Open depth stream
    VideoCapture depthStream(1620);
    // Open color stream
    VideoCapture colorStream(2, CAP_V4L2);
    //! [Open streams]

    // Check that stream has opened
    if (!colorStream.isOpened())
    {
        cerr << "ERROR: Unable to open color stream" << endl;
        return 1;
    }

    // Check that stream has opened
    if (!depthStream.isOpened())
    {
        cerr << "ERROR: Unable to open depth stream" << endl;
        return 1;
    }

    //! [Setup streams]
    // Set color and depth stream parameters
    colorStream.set(CAP_PROP_FRAME_WIDTH,  640);
    colorStream.set(CAP_PROP_FRAME_HEIGHT, 480);
    depthStream.set(CAP_PROP_FRAME_WIDTH,  640);
    depthStream.set(CAP_PROP_FRAME_HEIGHT, 480);
    depthStream.set(CAP_PROP_OPENNI2_MIRROR, 0);
    //! [Setup streams]

    // Print color stream parameters
    cout << "Color stream: "
         << colorStream.get(CAP_PROP_FRAME_WIDTH) << "x" << colorStream.get(CAP_PROP_FRAME_HEIGHT)
         << " @" << colorStream.get(CAP_PROP_FPS) << " fps" << endl;

    //! [Get properties]
    // Print depth stream parameters
    cout << "Depth stream: "
         << depthStream.get(CAP_PROP_FRAME_WIDTH) << "x" << depthStream.get(CAP_PROP_FRAME_HEIGHT)
         << " @" << depthStream.get(CAP_PROP_FPS) << " fps" << endl;
    //! [Get properties]

    //! [Read streams]
    // Create two lists to store frames
    std::list<Frame> depthFrames, colorFrames;
    const std::size_t maxFrames = 64;

    // Synchronization objects
    std::mutex mtx;
    std::condition_variable dataReady;
    std::atomic<bool> isFinish;

    isFinish = false;

    // Start depth reading thread
    std::thread depthReader([&]
    {
        while (!isFinish)
        {
            // Grab and decode new frame
            if (depthStream.grab())
            {
                Frame f;
                f.timestamp = cv::getTickCount();
                depthStream.retrieve(f.frame, CAP_OPENNI_DEPTH_MAP);
                if (f.frame.empty())
                {
                    cerr << "ERROR: Failed to decode frame from depth stream" << endl;
                    break;
                }

                {
                    std::lock_guard<std::mutex> lk(mtx);
                    if (depthFrames.size() >= maxFrames)
                        depthFrames.pop_front();
                    depthFrames.push_back(f);
                }
                dataReady.notify_one();
            }
        }
    });

    // Start color reading thread
    std::thread colorReader([&]
    {
        while (!isFinish)
        {
            // Grab and decode new frame
            if (colorStream.grab())
            {
                Frame f;
                f.timestamp = cv::getTickCount();
                colorStream.retrieve(f.frame);
                if (f.frame.empty())
                {
                    cerr << "ERROR: Failed to decode frame from color stream" << endl;
                    break;
                }

                {
                    std::lock_guard<std::mutex> lk(mtx);
                    if (colorFrames.size() >= maxFrames)
                        colorFrames.pop_front();
                    colorFrames.push_back(f);
                }
                dataReady.notify_one();
            }
        }
    });
    //! [Read streams]
    ofstream fout;
    string  filename = "Association.txt";
    fout.open(filename.c_str());
    //! [Pair frames] 帧同步
    // Pair depth and color frames
    int i=1;
    while (!isFinish)
    {
        std::unique_lock<std::mutex> lk(mtx);
        while (!isFinish && (depthFrames.empty() || colorFrames.empty()))
            dataReady.wait(lk);
        while (!depthFrames.empty() && !colorFrames.empty())
        {
            if (!lk.owns_lock())
                lk.lock();

            // Get a frame from the list
            Frame depthFrame = depthFrames.front();
            int64 depthT = depthFrame.timestamp;

            // Get a frame from the list
            Frame colorFrame = colorFrames.front();
            int64 colorT = colorFrame.timestamp;

            // Half of frame period is a maximum time diff between frames
            const int64 maxTdiff = int64(1000000000 / (2 * colorStream.get(CAP_PROP_FPS)));
            if (depthT + maxTdiff < colorT)
            {
                depthFrames.pop_front();
                continue;
            }
            else if (colorT + maxTdiff < depthT)
            {
                colorFrames.pop_front();
                continue;
            }
            depthFrames.pop_front();
            colorFrames.pop_front();
            lk.unlock();

            //! [Show frames]
            // Show depth frame
            Mat d8, dColor;
            //将深度图像转彩色显示
            depthFrame.frame.convertTo(d8, CV_8U, 255.0 / 2500);
            applyColorMap(d8, dColor, COLORMAP_OCEAN);//将深度图转为彩色显示
            imshow("Depth (colored)", dColor);
            imshow("Color", colorFrame.frame);
            char key=(char)waitKey(1);
            switch (key)
            {
            case 27:
            case 'q':
                isFinish = true;
                fout.close();
                break;
            case ' ':
            sprintf(DepthimgName, "./depth/Depth_%d.png",i);
            imwrite(DepthimgName,depthFrame.frame);//存储深度图
            sprintf(ColorimgName,"./rgb/Color_%d.png",i);
            imwrite(ColorimgName,colorFrame.frame);
            cout << "捕捉到第"<<i<<"组图像" << endl;
            i++;
            default:
                break;
            }
        }
    }
    //! [Pair frames]

    dataReady.notify_one();
    depthReader.join();
    colorReader.join();

    return 0;
}

