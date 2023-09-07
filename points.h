
#ifndef LISTENER_POINTS_H
#define LISTENER_POINTS_H

#include <mutex>
#include <vector>
#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

#include <utility>
using namespace std;
template <typename T>
class Points {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
     Points(
         Eigen::Vector3f color=Eigen::Vector3f{1,0,0},
         uint64_t time = 0.0,
         string frame="",
         double size=1.);
     Points(vector<T> &input,
           Eigen::Vector3f color=Eigen::Vector3f{1,0,0},
           uint64_t time = 0.0,
           string frame="",
           double size=1.);
    ~Points();
    
  public:
    bool empty() {return m_points_.empty();};
    void drawPoints();
    void setSize(double size) { msize_ = size; }
    void clear()               { vector<T>().swap(m_points_); }
    void addPoint(T point);
    void addPoints( vector<T> &input );
    auto getFrame(){ return mframe_; };
    void setFrame( string &frame){ mframe_ = frame; }
    uint64_t getTime(){return mTime_;}
    void setTime(uint64_t time){mTime_ = time;}
  private:
    mutex     mlock_;
    double    msize_;
    string    mframe_;
    vector<T> m_points_;
    uint64_t  mTime_;
    Eigen::Vector3f mcolor_;
};
template <typename T>
Points<T>::Points(Eigen::Vector3f color, uint64_t time, string frame, double size)
    :mTime_(time),mcolor_(std::move(color)),mframe_(std::move(frame)),msize_(size) {
    
}

template <typename T>
Points<T>::Points(vector<T> &input,Eigen::Vector3f color, uint64_t time, string frame, double size)
    :mTime_(time),mcolor_(std::move(color)),mframe_(std::move(frame)),msize_(size) {
    lock_guard< mutex> lockGuard(mlock_);
    m_points_.assign(input.begin(), input.end());
}


template <typename T>
Points<T>::~Points() {
    clear();
}
template <typename T>
void Points<T>::addPoint(T point) {
    lock_guard< mutex> lockGuard(mlock_);
    m_points_.emplace_back(point);
}

template <typename T>
void Points<T>::addPoints( vector<T> &input) {
    lock_guard< mutex> lockGuard(mlock_);
    m_points_.insert(m_points_.end(), input.begin(), input.end());
}

template <typename T>
void Points<T>::drawPoints() {
    if(m_points_.empty()) return;
     lock_guard< mutex> lockGuard(mlock_);
    glPointSize(msize_);
    glColor3f(mcolor_[0],mcolor_[1],mcolor_[2]);
    glBegin(GL_POINTS);
    for (const auto &pt: m_points_) {
      glVertex3f(pt[0], pt[1], pt[2]);
    }
    glEnd();
}
#endif //LISTENER_POINTS_H
