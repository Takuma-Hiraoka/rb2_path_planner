#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace wholebodycontact_locomotion_planner_sample{
  void sample0_display();
  class sample0_displayItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample0_displayItem>("sample0_displayItem"); }
  protected:
    virtual void main() override{ sample0_display(); return;}
  };
  typedef cnoid::ref_ptr<sample0_displayItem> sample0_displayItemPtr;

  void sample1_walk();
  class sample1_walkItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_walkItem>("sample1_walkItem"); }
  protected:
    virtual void main() override{ sample1_walk(); return;}
  };
  typedef cnoid::ref_ptr<sample1_walkItem> sample1_walkItemPtr;

  class WholeBodyContactLocomotionPlannerSamplePlugin : public cnoid::Plugin
  {
  public:
    WholeBodyContactLocomotionPlannerSamplePlugin() : Plugin("WholeBodyContactLocomotionPlannerSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample0_displayItem::initializeClass(this);
      sample1_walkItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(wholebodycontact_locomotion_planner_sample::WholeBodyContactLocomotionPlannerSamplePlugin)
