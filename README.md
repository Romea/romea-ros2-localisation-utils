# romea_localisation_utils

`romea_localisation_utils` provides ROS2 helper APIs used to connect localisation plugin nodes and localisation core filters.

It is not a runtime node package. It contains C++ utilities for:

* converting between `romea_localisation_msgs` messages and `romea_core_localisation` observation types;
* declaring and reading common localisation filter and updater parameters;
* creating debug topic loggers for predictors and updaters;
* creating localisation updater interfaces that subscribe to ROS2 observation topics and feed core localisation filters.

## 1) Role in the localisation stack

Localisation plugins publish typed observations as ROS2 messages. Localisation core nodes subscribe to these messages and fuse them with filters from `romea_core_localisation`. This package provides the reusable glue between both layers.

```mermaid
flowchart LR
  subgraph ros2_messages["romea_localisation_msgs"]
    msg["Observation messages"]
  end

  subgraph utils["romea_localisation_utils"]
    conversions["Conversions<br/>ROS2 messages <-> core types"]
    parameters["Parameters<br/>filters and updaters"]
    loggers["Loggers<br/>debug topic loggers"]
    interfaces["UpdaterInterfaces<br/>deferred subscriptions and filter updates"]
  end

  subgraph core["romea_core_localisation"]
    observations["Core observations"]
    filters["Core filter assemblies"]
  end

  msg -->|convert| conversions
  conversions -->|produce| observations
  parameters -->|configure| filters
  loggers -->|debug| filters
  msg -->|subscribe| interfaces
  interfaces -->|feed| filters
  observations -->|update| filters

  classDef ros2 fill:#e8f2ff,stroke:#5b8ec7,color:#111,rx:6,ry:6
  classDef utilsStyle fill:#eaf7ea,stroke:#5c9f5c,color:#111,rx:6,ry:6
  classDef coreStyle fill:#f1eaff,stroke:#8b6fc6,color:#111,rx:6,ry:6

  class msg ros2
  class conversions,parameters,loggers,interfaces utilsStyle
  class observations,filters coreStyle

  style ros2_messages fill:#f6faff,stroke:#9abbe3,rx:6,ry:6
  style utils fill:#f7fff7,stroke:#9ecf9e,rx:6,ry:6
  style core fill:#faf7ff,stroke:#b8a4dd,rx:6,ry:6
```

## 2) Message conversions

The `conversions/` headers convert localisation messages to and from the corresponding ROMEA core data structures.

| Header | Main purpose |
| ------ | ------------ |
| `observation_angular_speed_conversions.hpp` | Converts angular speed observations. |
| `observation_attitude_conversions.hpp` | Converts attitude observations. |
| `observation_course_conversions.hpp` | Converts course observations. |
| `observation_linear_speed_conversions.hpp` | Converts single linear speed observations. |
| `observation_linear_speeds_conversions.hpp` | Converts left/right linear speed observations. |
| `observation_pose_conversions.hpp` | Converts 2D pose observations. |
| `observation_position_conversions.hpp` | Converts 2D position observations. |
| `observation_range_conversions.hpp` | Converts range observations. |
| `observation_twist_conversions.hpp` | Converts 2D twist observations. |

All observation conversions preserve the observation timestamp, frame information and uncertainty carried by the message.

## 3) Localisation parameters

The `filter/parameters.hpp` API declares and reads the parameter groups shared by localisation filters and updater interfaces.

### 3.1) Filter and predictor parameters

| Parameter | Used by | Description |
| --------- | ------- | ----------- |
| `filter.state_pool_size` | Kalman and particle assemblies | Size of the internal filter state pool. |
| `filter.number_of_particles` | Particle assemblies | Number of particles used by particle filter implementations. |
| `predictor.maximal_dead_recknoning_travelled_distance` | Predictors | Maximum travelled distance allowed in dead-reckoning mode. |
| `predictor.maximal_dead_recknoning_elapsed_time` | Predictors | Maximum elapsed time allowed in dead-reckoning mode. |

### 3.2) Updater parameters

| Parameter | Used by | Description |
| --------- | ------- | ----------- |
| `<updater>.minimal_rate` | Proprioceptive and exteroceptive updaters | Minimal expected observation rate. A value of `0` disables the updater at node configuration level; constructed core updaters require a strictly positive rate. |
| `<updater>.trigger` | Exteroceptive updaters | Update trigger mode, either `always` or `once`. |
| `<updater>.mahalanobis_distance_rejection_threshold` | Exteroceptive updaters | Innovation rejection threshold used to discard outliers. |

## 4) Debug loggers

The `filter/loggers.hpp` API creates topic loggers used by core predictors and updaters when the node `debug` parameter is enabled.

| Helper | Purpose |
| ------ | ------- |
| `make_topic_logger(node, logger_name)` | Returns a `TopicLogger` publishing on `debug/<logger_name>` when `debug=true`; otherwise returns `nullptr`. |

Predictors and updaters are now created by `romea_core_localisation::Filter`, while this package keeps ROS2-side parameter handling, message conversion, debug loggers and updater interfaces reusable.

## 5) Updater interfaces

`UpdaterInterface` subscribes to one ROS2 observation topic and forwards each received observation through a callback returned by the core localisation filter.

For each message:

1. the timestamp is extracted from the ROS2 message;
2. the message is converted to the corresponding core observation;
3. the stored update callback sends the observation to the core filter at the observation timestamp.

The `UpdaterInterfaces` manager stores interface builders while the core filter is configured. After all updaters have been added and the core filter has been initialized, `UpdaterInterfaces::create()` instantiates the ROS2 subscriptions. This ordering avoids callbacks reaching a filter before its predictor and updater constraints are fully configured.

## 6) Typical use

A localisation node usually combines the utilities in this order:

```cpp
declare_filter_parameters<core::FilterType::KALMAN>(node);
declare_predictor_parameters(node, 2.0, 10.0);

using Traits = core::localisation::R2WTraits<core::FilterType::KALMAN>;
using Filter = core::localisation::Filter<core::FilterType::KALMAN, Traits>;

auto filter = std::make_unique<Filter>(
  get_filter_state_pool_size(node),
  get_dead_reckoning_limits(node));

auto callback = filter->add_proprioceptive_updater<Traits::UpdaterTwist>(
  "twist_updater",
  get_updater_minimal_rate(node, "twist_updater"));

UpdaterInterfaces updater_interfaces;
if (callback) {
  updater_interfaces.add<UpdaterInterfaceTwist>(
    "twist_updater", "twist", std::move(*callback));
}

Filter::LoggerMap loggers = {
  {"predictor", make_topic_logger(node, "predictor")},
  {"twist_updater", make_topic_logger(node, "twist_updater")}};

filter->initialize(
  event_log_callback,
  get_results_log_callback,
  fsm_event_callback,
  loggers);
updater_interfaces.create(node);
```

Concrete localisation core packages hide this boilerplate behind architecture-specific filter classes. They add all updaters first, initialize the filter, and only then create the ROS2 subscriptions through `UpdaterInterfaces::create()`.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This package was developed by **Jean Laneurit** in the context of the BaudetRob2 ANR project.
