#include<nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "test_nodelets/comp_param.h"

namespace calculators {

  class CompParamNodelet : public nodelet::Nodelet {
  public:
    CompParamNodelet(){}
    ~CompParamNodelet(){}

    virtual void onInit()
    {
      ros::NodeHandle nh = this->getPrivateNodeHandle();
      std::string name = nh.getUnresolvedNamespace();
      name = name.substr(name.find_last_of('/') + 1);

      NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
      comp_param_.reset(new CompParamClass(nh, name));
      comp_param_->init();
    }
  private:
      boost::shared_ptr<CompParamClass> comp_param_;

  };

}

PLUGINLIB_EXPORT_CLASS(calculators::CompParamNodelet,
                       nodelet::Nodelet);
