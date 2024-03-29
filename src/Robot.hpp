#pragma once
#include "Cartesian2DPoint.hpp"
#include "CellData.hpp"
#include <deque>
#include <thread>

using namespace std;
class Robot : public std::enable_shared_from_this<Robot>
{
public:
    Robot(int id, shared_ptr<CellData> cell) : _id(id)
    {
        _position.x = cell->cartesianPosition.x;
        _position.y = cell->cartesianPosition.y;
        setNewPrkingCell(cell);
    };
    Robot(int id, shared_ptr<CellData> cell, double radius) : _id(id), _radius(radius)
    {
        _position.x = cell->cartesianPosition.x;
        _position.y = cell->cartesianPosition.y;
        setNewPrkingCell(cell);
    };
    bool goalReached = false;
    std::mutex mtx;

    shared_ptr<Robot> trackPath()
    {
        while (isBusy())
        {
            auto goal = _path.front()->cartesianPosition;
            setGoal(goal);
            goalReached = false;
            int n = 500;
            double stepingDistance = this->distanceToPoint(goal) / n;
            auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < n; i++)
            {
                auto start_time = std::chrono::steady_clock::now();
                double cos_heading = (goal.y - _position.y) / distanceToPoint(goal);
                double sin_heading = (goal.x - _position.x) / distanceToPoint(goal);
                double dy = cos_heading * stepingDistance;
                double dx = sin_heading * stepingDistance;
                step(dx, dy);
                auto end_time = start_time + _Frame_duration(1);
                std::this_thread::sleep_until(end_time);
            }
            goalReached = true;
            std::lock_guard<std::mutex> lck(mtx);
            _path.pop_front();
        }
        return shared_from_this();
    }

    void step(double dx, double dy)
    {
        _position.x = _position.x + dx;
        _position.y = _position.y + dy;
    }

    double distanceToPoint(Cartesian2DPoint point)
    {
        double x1 = _position.x;
        double y1 = _position.y;
        double x2 = point.x;
        double y2 = point.y;
        return sqrt(pow(x2 - x1, 2) +
                    pow(y2 - y1, 2));
    }

    Cartesian2DPoint getGoal()
    {
        return _goal;
    };

    double getRadius()
    {
        return _radius;
    };

    void setGoal(Cartesian2DPoint g)
    {
        _goal = g;
    };

    void setNewPrkingCell(shared_ptr<CellData> cell)
    {

        resetParkingCell();
        std::lock_guard<std::mutex> lck(mtx);
        _parkingCell = cell;
        _parkingCell->aRobotIsParkingHere = true;
    }

    void resetParkingCell()
    {
        std::lock_guard<std::mutex> lck(mtx);
        if (_parkingCell != nullptr)
            _parkingCell->aRobotIsParkingHere = false;
    }

    void appendCellToPath(std::shared_ptr<CellData> cell, int t0)
    {
        std::lock_guard<std::mutex> lck(mtx);
        cell->reserveCell();
        _path.push_front(cell);
    }

    shared_ptr<CellData> getParkingCell()
    {
        std::lock_guard<std::mutex> lck(mtx);
        return _parkingCell;
    }
    Cartesian2DPoint getPosition()
    {
        std::lock_guard<std::mutex> lck(mtx);
        return _position;
    };

    int getID()
    {
        std::lock_guard<std::mutex> lck(mtx);
        return _id;
    }

    std::deque<std::shared_ptr<CellData>> getPath()
    {
        std::lock_guard<std::mutex> lck(mtx);
        return _path;
    };

    bool isNotBusy()
    {
        std::lock_guard<std::mutex> lck(mtx);

        return _path.empty();
    }

    bool isBusy()
    {
        std::lock_guard<std::mutex> lck(mtx);

        return !_path.empty();
    }

private:
    Cartesian2DPoint _position;
    int _id = 0;
    Cartesian2DPoint _goal = Cartesian2DPoint(_position.x, _position.y);
    double _radius = 10;
    std::deque<std::shared_ptr<CellData>> _path;
    shared_ptr<CellData> _parkingCell;
    thread t_;
    typedef std::chrono::duration<int, std::ratio<1, 520>> _Frame_duration;
};
