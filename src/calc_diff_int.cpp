#include<nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "test_nodelets/calc_diff_int.hpp"

namespace calculators {

class CompIntTemporalNodelet : public nodelet::Nodelet {
public:
  CompIntTemporalNodelet() {}
  ~CompIntTemporalNodelet() {}

  virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        std::string name = nh.getUnresolvedNamespace();
        name = name.substr(name.find_last_of('/') + 1);

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        comp_temporal_.reset(new CalcDiffInt(nh, name));
        comp_temporal_->init();
      }

private:
    boost::shared_ptr<CalcDiffInt> comp_temporal_;
};

}

PLUGINLIB_EXPORT_CLASS(calculators::CompIntTemporalNodelet,
                       nodelet::Nodelet);
