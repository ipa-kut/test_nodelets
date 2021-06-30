#include<nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "test_nodelets/comp_float_temporal.h"

namespace calculators {

  class CompFloatTemporalNodelet : public nodelet::Nodelet {
  public:
    CompFloatTemporalNodelet(){}
    ~CompFloatTemporalNodelet(){}

    virtual void onInit()
    {
      ros::NodeHandle nh = this->getMTPrivateNodeHandle();
      std::string name = nh.getUnresolvedNamespace();
      name = name.substr(name.find_last_of('/') + 1);

      NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
      comp_temporal_.reset(new CompFloatTemporalClass(nh, name));
      comp_temporal_->init();
    }
  private:
      boost::shared_ptr<CompFloatTemporalClass> comp_temporal_;

  };

}

PLUGINLIB_EXPORT_CLASS(calculators::CompFloatTemporalNodelet,
                       nodelet::Nodelet);
