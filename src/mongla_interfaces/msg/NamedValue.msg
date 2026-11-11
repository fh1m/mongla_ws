# One NAMED_VALUE_FLOAT channel. value is NaN once it is older than the
# manager's limit: a channel that stopped must not read as its last value.
string name
float32 value
float32 age_s
