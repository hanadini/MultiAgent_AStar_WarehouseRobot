#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "CellData.hpp"
#include "Warehouse.hpp"
#include "Robot.hpp"
#include "Graphics.hpp"
#include "Planner.hpp"
#include "GenericQueue.hpp"
#include <vector>
#include <deque>
#include <thread>
#include <future>
#include <random>

using namespace cv;

const int MAX_MONITOR_LENGTH = 1080;
const int MAX_MONITOR_WIDTH = 1920;

std::mutex mtx;

void planningThread(shared_ptr<GenericQueue<shared_ptr<Robot>>> avialableRobots, shared_ptr<deque<shared_ptr<Robot>>> busyRobots, Map *map)
{

    deque<pair<shared_ptr<CellData>, shared_ptr<CellData>>> tasks;
    for(int j=18;j<=26;j++){
        tasks.emplace_back(map->getCell(5, j), map->getCell(0, 0));
    }

    for(int j=18;j<=26;j++){
        tasks.emplace_back(map->getCell(3, j), map->getCell(0, 0));
    }

    for(int j=7;j<=16;j++){
        tasks.emplace_back(map->getCell(7, j), map->getCell(0, 0));
    }

    for(int j=7;j<=16;j++){
        tasks.emplace_back(map->getCell(9, j), map->getCell(0, 0));
    }

    Planner multiAgentPlanner(map);
    int minDuration = 0;
    int maxDuration = 20;

    int t0 = 0;
    typedef std::chrono::duration<int, std::ratio<1, 1>> _Frame_duration;
    auto tic0 = std::chrono::steady_clock::now();
    while (true)
    {
        auto tic = std::chrono::steady_clock::now();
        shared_ptr<Robot> rob = avialableRobots->receive();
        unique_lock<mutex> uL(mtx);
        std::cout << "[Planning thread] recived robot #" << rob->getID() << std::endl;
        uL.unlock();
        if (!tasks.empty())
        {

            auto end_time = tic + _Frame_duration(1);
            std::this_thread::sleep_until(end_time);
            auto tStamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - tic0);
            t0 = tStamp.count();
            bool pathFound = multiAgentPlanner.planPath(rob, tasks.front(), t0);
            if(pathFound){
                tasks.pop_front();
            }
            uL.lock();
            busyRobots->push_back(std::move(rob));
            uL.unlock();
        }
    }
}

int main(int argc, char **argv)
{
    // visulize the warehouse
    Warehouse warehouse = Warehouse();
    double aspectRatio = 0.7;
    int windowWidth = int(aspectRatio * MAX_MONITOR_WIDTH);
    int windowLength = int(aspectRatio * MAX_MONITOR_LENGTH);
    Graphics viewer = Graphics(windowLength, windowWidth, warehouse._map);

    // construct a fleet of robots
    deque<shared_ptr<Robot>> fleet;
    for(int i=1;i<=19;i++){
        if (i%2 == 0){
            fleet.emplace_back(std::make_shared<Robot>(i, warehouse._map.getCell(i, 2), warehouse._map.getCellSize() * 0.5));
        }else{
            fleet.emplace_back(std::make_shared<Robot>(i, warehouse._map.getCell(i, 32), warehouse._map.getCellSize() * 0.5));
        }
    }

    for(int i=1;i<=19;i++){
        if (i%2 != 0){
            fleet.emplace_back(std::make_shared<Robot>(i+19, warehouse._map.getCell(i, 2), warehouse._map.getCellSize() * 0.5));
        }else{
            fleet.emplace_back(std::make_shared<Robot>(i+19, warehouse._map.getCell(i, 32), warehouse._map.getCellSize() * 0.5));
        }
    }

    // start viwer thread
    viewer.setRobots(fleet);
    viewer.loadBackgroundImg();
    std::thread simulationThread(&Graphics::run, &viewer);

    // wait for 1 sec before starting Planning and Execution
    this_thread::sleep_for(chrono::milliseconds(1000));

    shared_ptr<deque<shared_ptr<Robot>>> busyRobots = std::make_shared<deque<shared_ptr<Robot>>>();
    shared_ptr<GenericQueue<shared_ptr<Robot>>> availableRobots = std::make_shared<GenericQueue<shared_ptr<Robot>>>();

    for (auto robot : fleet)
    {
        availableRobots->send(std::move(robot));
    }

    // launch planning thread
    std::thread pThread(&planningThread, availableRobots, busyRobots, &(warehouse._map));
    fleet.clear();
    std::vector<std::future<shared_ptr<Robot>>> moveThread;

    // Execution Tread
    while (true)
    {
        unique_lock<mutex> uL(mtx);
        bool aRobotIsAvailable = !busyRobots->empty();
        uL.unlock();
        if (aRobotIsAvailable)
        {
            uL.lock();
            auto robot = std::move(busyRobots->front());
            busyRobots->pop_front();
            std::cout << "[Execution thread] recived robot #" << robot->getID() << std::endl;
            uL.unlock();
            moveThread.emplace_back(async(std::launch::async, &Robot::trackPath, robot));
        }
        for (int i = 0; i < moveThread.size(); i++)
        {
            if ((moveThread[i].wait_for(chrono::milliseconds(1))) == future_status::ready)
            {
                availableRobots->send(move(moveThread[i].get()));
                moveThread.erase(moveThread.begin() + i);
            };
        }

         this_thread::sleep_for(chrono::milliseconds(1));
    }

    std::cout << "exit" << std::endl;
    // wait for the user to press any key:
    simulationThread.join();
    pThread.join();
    waitKey(0);
    return 0;
}
