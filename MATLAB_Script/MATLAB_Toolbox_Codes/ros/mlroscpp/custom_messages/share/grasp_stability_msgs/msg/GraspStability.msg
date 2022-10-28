## All of these fields are subject to change as the discussion about
## integration of grasp stability estimation and CRAM progresses


# For telling different grasp stability estimations (i.e. multiple
# arms that are monitored simultaneously) apart.
string measurement_context_id

# Functional data, describing the results of the estimation, and
# the confidence in the result.
float32 grasp_quality		# range: 0.0 - 1.0
float32 estimation_confidence	# range: 0.0 - 1.0

# Grasp category, based on the above values
int32 GRASP_CAT_UNDEFINED=0
int32 GRASP_CAT_GOOD=1
int32 GRASP_CAT_MEDIUM=2
int32 GRASP_CAT_BAD=3

int32 grasp_category		# the category of the grasp