# romea_localisation_utils

`romea_localisation_utils` provides ROS2 helper APIs used to connect localisation plugin nodes and localisation core filters.

It is not a runtime node package. It contains C++ utilities for:

* converting between `romea_localisation_msgs` messages and `romea_core_localisation` observation types;
* declaring and reading common localisation filter and updater parameters;
* creating core filter assemblies from ROS2 parameters;
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
    factories["Factories<br/>filter components, predictors, updaters"]
    interfaces["Updater interfaces<br/>subscriptions and filter updates"]
  end

  subgraph core["romea_core_localisation"]
    observations["Core observations"]
    filters["Core filter assemblies"]
  end

  msg -->|convert| conversions
  conversions -->|produce| observations
  parameters -->|configure| factories
  factories -->|create| filters
  factories -->|create| interfaces
  interfaces -->|feed| filters
  observations -->|update| filters

  classDef ros2 fill:#e8f2ff,stroke:#5b8ec7,color:#111,rx:6,ry:6
  classDef utilsStyle fill:#eaf7ea,stroke:#5c9f5c,color:#111,rx:6,ry:6
  classDef coreStyle fill:#f1eaff,stroke:#8b6fc6,color:#111,rx:6,ry:6

  class msg ros2
  class conversions,parameters,factories,interfaces utilsStyle
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

The `filter/localisation_parameters.hpp` API declares and reads the parameter groups shared by localisation filters and updater interfaces.

### 3.1) Filter and predictor parameters

| Parameter | Used by | Description |
| --------- | ------- | ----------- |
| `filter.state_pool_size` | Kalman and particle assemblies | Size of the internal filter state pool. |
| `filter.number_of_particles` | Particle assemblies | Number of particles used by particle filter implementations. |
| `predictor.maximal_dead_recknoning_travelled_distance` | Predictors | Maximum travelled distance allowed in dead-reckoning mode. |
| `predictor.maximal_dead_recknoning_elapsed_time` | Predictors | Maximum elapsed time allowed in dead-reckoning mode. |
| `predictor.maximal_position_circular_error_probability` | Predictors | Maximum position uncertainty allowed in dead-reckoning mode. |

### 3.2) Updater parameters

| Parameter | Used by | Description |
| --------- | ------- | ----------- |
| `<updater>.minimal_rate` | Proprioceptive and exteroceptive updaters | Minimal expected observation rate. A value of `0` is commonly used by localisation nodes to disable an updater. |
| `<updater>.trigger` | Exteroceptive updaters | Update trigger mode, either `always` or `once`. |
| `<updater>.mahalanobis_distance_rejection_threshold` | Exteroceptive updaters | Innovation rejection threshold used to discard outliers. |

## 4) Factories

The `filter/localisation_factory.hpp` API creates core localisation components from ROS2 node parameters.

| Factory function family | Creates |
| ----------------------- | ------- |
| `make_filter` | A Kalman or particle filtering engine configured with the selected localisation state type. |
| `make_predictor` | A predictor configured with dead-reckoning limits. |
| `make_meta_state` | Meta-state containers matching the selected filter type. |
| `make_meta_state_to_results` | Converters from meta-state to public results. |
| `make_proprioceptive_updater` | Updaters fed by proprioceptive observations such as twist or angular speed. |
| `make_exteroceptive_updater` | Updaters fed by exteroceptive observations such as position, pose or range. |

The factory layer is templated so that localisation nodes can select the core filtering engine, predictor and updater types through their traits while keeping the ROS2 parameter handling reusable.

## 5) Updater interfaces

`LocalisationUpdaterInterface` subscribes to one ROS2 observation topic and forwards each received observation to a core localisation filter.

For each message:

1. the timestamp is extracted from the ROS2 message;
2. the message is converted to the corresponding core observation;
3. the core updater update function is bound with this observation;
4. the filter processes the update at the observation timestamp.

The interface also exposes heartbeat and diagnostic hooks so localisation nodes can report whether each configured observation stream is alive and arrives at the expected rate.

## 6) Typical use

A localisation node usually combines the utilities in this order:

```cpp
declare_filter_parameters<core::FilterType::KALMAN>(node);
declare_predictor_parameters(node, 2.0, 10.0, std::numeric_limits<double>::max());

auto filter = make_filter<Filter, Predictor, core::FilterType::KALMAN>(node);

auto updater = make_proprioceptive_updater<UpdaterTwist>(node, "twist_updater");

auto interface = make_updater_interface<UpdaterInterfaceTwist>(
  node,
  "twist",
  filter,
  std::move(updater));
```

Concrete localisation core packages usually hide this boilerplate behind architecture-specific filter classes.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This package was developed by **Jean Laneurit** in the context of the BaudetRob2 ANR project.
