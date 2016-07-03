#Name and unique ID for the temporal constraint
string name

#Names of the 'start' and 'end' events of the temporal constraint
string  start_event_name
string  end_event_name

#Type of temporal constraint:
# controllable, uncontrollable_bounded, uncontrollable_probabilistic
string  type

#List of properties in the form PROPERTY=VALUE
string[] properties
