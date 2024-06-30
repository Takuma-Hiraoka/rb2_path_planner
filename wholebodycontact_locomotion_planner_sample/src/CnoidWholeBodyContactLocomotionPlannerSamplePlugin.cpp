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

  void sample2_tunnel();
  class sample2_tunnelItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_tunnelItem>("sample2_tunnelItem"); }
  protected:
    virtual void main() override{ sample2_tunnel(); return;}
  };
  typedef cnoid::ref_ptr<sample2_tunnelItem> sample2_tunnelItemPtr;

  void sample3_jaxon();
  class sample3_jaxonItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample3_jaxonItem>("sample3_jaxonItem"); }
  protected:
    virtual void main() override{ sample3_jaxon(); return;}
  };
  typedef cnoid::ref_ptr<sample3_jaxonItem> sample3_jaxonItemPtr;

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
      sample2_tunnelItem::initializeClass(this);
      sample3_jaxonItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(wholebodycontact_locomotion_planner_sample::WholeBodyContactLocomotionPlannerSamplePlugin)
