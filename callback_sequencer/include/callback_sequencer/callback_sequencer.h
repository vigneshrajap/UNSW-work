
#ifndef CALLBACK_SEQUENCER_CALLBACK_SEQUENCER_H_
#define CALLBACK_SEQUENCER_CALLBACK_SEQUENCER_H_

#include <ros/ros.h>

#include <message_filters/connection.h>
#include <message_filters/null_types.h>

#include <boost/tuple/tuple.hpp>

namespace callback_sequencer
{

template <typename M0, typename M1, typename M2 = message_filters::NullType, typename M3 = message_filters::NullType>
class CallbackSequencer
{

  typedef boost::mpl::vector<M0, M1, M2, M3> Messages;

  typedef const boost::shared_ptr<const M0> M0ConstPtr;
  typedef const boost::shared_ptr<const M1> M1ConstPtr;
  typedef const boost::shared_ptr<const M2> M2ConstPtr;
  typedef const boost::shared_ptr<const M3> M3ConstPtr;

  typedef boost::mpl::vector<M0ConstPtr, M1ConstPtr, M2ConstPtr, M3ConstPtr> ConstPtrs;

  typedef boost::mpl::vector<ros::MessageEvent<const M0>, ros::MessageEvent<const M1>, ros::MessageEvent<const M2>, ros::MessageEvent<const M3> > Events;
  typedef typename boost::mpl::at_c<Events, 0>::type M0Event;
  typedef typename boost::mpl::at_c<Events, 1>::type M1Event;
  typedef typename boost::mpl::at_c<Events, 2>::type M2Event;
  typedef typename boost::mpl::at_c<Events, 3>::type M3Event;

  typedef boost::tuple<ros::Time, int> TimeStampIdx;

public:

  // Default constructor.
  CallbackSequencer(double update_rate = 0.1, double delay = 0.1) : input_connections_(4), update_rate_(update_rate), delay_(delay)
  {
    init();
  }

  // Constructor for 2 inputs.
  template <typename F0, typename C0, typename F1, typename C1>
  CallbackSequencer(F0 & f0, C0 & c0, F1 & f1, C1 & c1, double update_rate = 0.1, double delay = 0.1) : input_connections_(4), update_rate_(update_rate), delay_(delay)
  {
    connectInput(f0, c0, f1, c1);
    init();
  }

  // Constructor for 3 inputs.
  template <typename F0, typename C0, typename F1, typename C1, typename F2, typename C2>
  CallbackSequencer(F0 & f0, C0 & c0, F1 & f1, C1 & c1, F2 & f2, C2 & c2, double update_rate = 0.1, double delay = 0.1) : input_connections_(4), update_rate_(update_rate), delay_(delay)
  {
    connectInput(f0, c0, f1, c1, f2, c2);
    init();
  }

  // Constructor for 4 inputs.
  template <typename F0, typename C0, typename F1, typename C1, typename F2, typename C2, typename F3, typename C3>
  CallbackSequencer(F0 & f0, C0 & c0, F1 & f1, C1 & c1, F2 & f2, C2 & c2, F3 & f3, C3 & c3, double update_rate = 0.1, double delay = 0.1) : input_connections_(4), update_rate_(update_rate), delay_(delay)
  {
    connectInput(f0, c0, f1, c1, f2, c2, f3, c3);
    init();
  }

  // Connect 2 inputs.
  template<typename F0, typename C0, typename F1, typename C1>
  void connectInput(F0 & f0, C0 & c0, F1 & f1, C1 & c1)
  {
    message_filters::NullFilter<M2> f2;
    boost::function<void (M2ConstPtr &)> c2;
    connectInput(f0, c0, f1, c1, f2, c2);
  }

  template<typename F0, typename C0, typename F1, typename C1, typename F2, typename C2>
  void connectInput(F0 & f0, C0 & c0, F1 & f1, C1 & c1, F2 & f2, C2 & c2)
  {
    message_filters::NullFilter<M3> f3;
    boost::function<void (M3ConstPtr)> c3;
    connectInput(f0, c0, f1, c1, f2, c2, f3, c3);
  }

  template <typename F0, typename C0, typename F1, typename C1, typename F2, typename C2, typename F3, typename C3>
  void connectInput(F0 & f0, C0 & c0, F1 & f1, C1 & c1, F2 & f2, C2 & c2, F3 & f3, C3 & c3)
  {

    input_connections_.at(0) = f0.registerCallback(boost::function<void (const M0Event &)>(boost::bind(&CallbackSequencer::callback<0>, this, _1)));
    input_connections_.at(1) = f1.registerCallback(boost::function<void (const M1Event &)>(boost::bind(&CallbackSequencer::callback<1>, this, _1)));
    input_connections_.at(2) = f2.registerCallback(boost::function<void (const M2Event &)>(boost::bind(&CallbackSequencer::callback<2>, this, _1)));
    input_connections_.at(3) = f3.registerCallback(boost::function<void (const M3Event &)>(boost::bind(&CallbackSequencer::callback<3>, this, _1)));

    boost::get<0>(callbacks_).registerCallback(boost::function<void (M0ConstPtr &)>(c0));
    boost::get<1>(callbacks_).registerCallback(boost::function<void (M1ConstPtr &)>(c1));
    boost::get<2>(callbacks_).registerCallback(boost::function<void (M2ConstPtr &)>(c2));
    boost::get<3>(callbacks_).registerCallback(boost::function<void (M3ConstPtr &)>(c3));

  }

  void init()
  {
    update_timer_ = nh_.createTimer(update_rate_, &CallbackSequencer::update, this);
  }

  void setUpdateRate(const ros::Duration & update_rate)
  {
    update_rate_ = update_rate;
    update_timer_ = nh_.createTimer(update_rate_, &CallbackSequencer::update, this);
  }

  void setDelay(const ros::Duration & delay)
  {
    delay_ = delay;
  }

  template <int i, typename F, typename C>
  void subscribe(F & f, const C & c)
  {
    input_connections_.at(i) = f.registerCallback(boost::function<void (const typename boost::mpl::at_c<Events, i>::type &)>(boost::bind(&CallbackSequencer::callback<i>, this, _1)));
    boost::get<i>(callbacks_).registerCallback(boost::function<void (typename boost::mpl::at_c<ConstPtrs, i>::type &)>(c));
  }

private:

  void update(const ros::TimerEvent &)
  {
    dispatch();
  }

  void dispatch()
  {

    std::vector<TimeStampIdx> to_call;

    while (!time_stamp_indices_.empty())
    {

      if (boost::get<0>(*time_stamp_indices_.begin()) + delay_ <= ros::Time::now())
      {
        to_call.push_back(*time_stamp_indices_.begin());
        time_stamp_indices_.erase(time_stamp_indices_.begin());
      }
      else
      {
        break;
      }
          
    }

    for (std::vector<TimeStampIdx>::iterator it = to_call.begin(); it != to_call.end(); ++it)
    {
      switch(boost::get<1>(*it))
      {
        case 0:
          boost::get<0>(callbacks_).signalMessage2(boost::get<0>(events_).begin()->getMessage());
          boost::get<0>(events_).erase(boost::get<0>(events_).begin());
          break;
        case 1:
          boost::get<1>(callbacks_).signalMessage2(boost::get<1>(events_).begin()->getMessage());
          boost::get<1>(events_).erase(boost::get<1>(events_).begin());
          break;
        case 2:
          boost::get<2>(callbacks_).signalMessage2(boost::get<2>(events_).begin()->getMessage());
          boost::get<2>(events_).erase(boost::get<2>(events_).begin());
          break;
        case 3:
          boost::get<3>(callbacks_).signalMessage2(boost::get<3>(events_).begin()->getMessage());
          boost::get<3>(events_).erase(boost::get<3>(events_).begin());
          break;
      }
    }

  }
  
  template <int i>
  void callback(const typename boost::mpl::at_c<Events, i>::type & event)
  {
    add<i>(event);
  }

  template <int i>
  void add(const typename boost::mpl::at_c<Events, i>::type & event)
  {
    //time_stamp_indices_.insert(TimeStampIdx(event.getMessage()->header.stamp, i));
    time_stamp_indices_.insert(TimeStampIdx(ros::message_traits::TimeStamp<typename boost::mpl::at_c<Messages, i>::type>::value(*event.getMessage()), i));
    boost::get<i>(events_).push_back(event);
  }

  template <typename M>
  class SimpleFilter : public message_filters::SimpleFilter<M>
  {

  typedef boost::shared_ptr<const M> MConstPtr;

  public:

    void signalMessage2(const MConstPtr & msg)
    {
      signalMessage(msg);
    }

  };

  class TimeStampSort
  {

  public:

    bool operator()(const TimeStampIdx & lhs, const TimeStampIdx & rhs) const
    {
      return boost::get<0>(lhs) < boost::get<0>(rhs);
    }

  };

  ros::NodeHandle nh_;

  std::vector<message_filters::Connection> input_connections_;

  std::multiset<TimeStampIdx, TimeStampSort> time_stamp_indices_;

  boost::tuple<std::vector<M0Event>, std::vector<M1Event>, std::vector<M2Event>, std::vector<M3Event> > events_;

  boost::tuple<SimpleFilter<M0>, SimpleFilter<M1>, SimpleFilter<M2>, SimpleFilter<M3> > callbacks_;

  ros::Timer update_timer_;

  ros::Duration update_rate_;

  ros::Duration delay_;

};

} // namespace callback_sequencer

#endif // #ifndef CALLBACK_SEQUENCER_CALLBACK_SEQUENCER_H_
