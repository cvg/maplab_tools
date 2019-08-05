#ifndef VI_MAP_VISUALIZATION_PLUGIN_VISUALIZATION_PLUGIN_H_
#define VI_MAP_VISUALIZATION_PLUGIN_VISUALIZATION_PLUGIN_H_

#include <memory>

#include <Eigen/Dense>
#include <console-common/console-plugin-base.h>
#include <console-common/console.h>
#include <string>
#include <vi-map/vi-map.h>
#include <map-manager/map-manager.h>

namespace common {
class Console;
}  // namespace common

namespace vi_visualization {

class VisualizationPlugin : public common::ConsolePluginBase {
 public:
  explicit VisualizationPlugin(common::Console* console);

  virtual std::string getPluginId() const {
    return "visualization";
  }

 private:
  void plotVIStatesOfMission(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id) const;
  int visualizeCvMatResources(backend::ResourceType type);
  int visualizeSensorExtrinsics() const;
  int visualizeLandmarkObserverRays() const;
  int visualizeReprojectedDepthResource(backend::ResourceType type);
  int visualizeArtifactDetections() const;

  int getAllMissionIds(
      vi_map::VIMapManager::MapWriteAccess& map,
      vi_map::MissionIdList* mission_ids) const;

};

}  // namespace vi_visualization

#endif  // VI_MAP_VISUALIZATION_PLUGIN_VISUALIZATION_PLUGIN_H_
