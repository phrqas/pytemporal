#Name of the temporal constraint
string name

#Lower and upper bounds coming from the temporal constraint model
float32 lb_model
float32 ub_model

#Lower and upper bounds imposed by the strong schedule
float32 lb_imposed
float32 ub_imposed

#Risk corresponding to modifying the temporal constraint bounds
float32 squeeze_risk
