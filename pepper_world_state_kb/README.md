# Updating the KB from topics

In this package you have a script that is able to listen to arbitrary topics and take that information to create knowledge (instances and predicates with arguments) and adds it to the knowledge base of rosplan.

This is achieved using the provided configuration file where you can define
static instances like
```
static_instances:
distance:
- arguments:
  - value: "close"
- arguments:
  - value: "far"
```

which results in two instances of type `distance`: `close` and `far` that are added to the knowledgebase when the script is started.

Then there is the `inputs` which define a topic for the message and a `base_attribute`.

```
inputs:
- topic: "/naoqi_driver_node/people_detected"
  base_attribute: "person_array"
```

`inputs` is a list so the script can subscribe to as many topics a necessary. `topic` defines the name of the topic and the type is infered once the topic is subscribed to.

Looking at the people perception example from NAOqi, the `base_attribute` is an array of detected people in that message like `for p in person_array`, so everything defined under `data` will be done for each of these `p`.

In `data` you define if it is an instance (for example add a new instance of type `id` for each detected person) or if it is a predicate. An example for an instances is:

```
instances:
id: 
  arguments:
    - format: "id_%s"
      attribute: "id"
```

Which results in a new instance of type `id` taken from `p.id` of the form `id_821` for example. The same principle applies to predicates. Predicates, however, also have a truth value which defines if the predicate should be `true` or `false` depending on the content of the message. Currently the logic here is rather primitive but here is how it works:

```
predicates:
      face_detected:
        arguments:
          - format: "id_%s"
            attribute: "id"
        truth_value:
          attribute: "person.face_detected"
          comparison: "$attribute == True"
```

This creates a new predicate `(face_detected id_821)` if the expression `p.person.face_detected == True` is true. So you see what is replaced here. If the expression is not evaluated to true it is automatically set to false with the next incoming message and removed from the knowledgebase. You can also use this to create simple qualitative representations like so:

```
      robot_distance:
        - arguments:
          - format: "id_%s"
            attribute: "id"
          - value: "close"
          truth_value:
            attribute: "person.distance"
            comparison: "$attribute > 0.0 and $attribute < 1.0"
        - arguments:
          - format: "id_%s"
            attribute: "id"
          - value: "far"
          truth_value:
            attribute: "person.distance"
            comparison: "$attribute >= 1.0"
```

This defines a list of possible predicates robot_distance that are all evaluated. So if `p.person.distance > 0.0 and p.person.distance < 1.0` is true the predicate `(robot_distance id_821 close)` is added to the knowledge base and since `p.person.distance >= 1.0` is evaluated to false in this case, `(robot_distance id_821 far)` would automatically be removed if it was in the DB.

If your message does not contain an array of people but is just a normal message with a few fileds (e.g. the `battery_state` topic) you can define the `base_attribute` as the empty string:

```
  - topic: "/naoqi_driver_node/battery"
    base_attribute: ""
```

Then all the attributes are directly evaluated on the highest level.