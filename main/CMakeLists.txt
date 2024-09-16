idf_component_register(
    SRCS
   "HA_custom_co2.c"
    INCLUDE_DIRS "."
)
set(ulp_app_name lp_core_${COMPONENT_NAME})
set(ulp_lp_core_sources "lp_core/main.c")
set(ulp_exp_dep_srcs "HA_custom_co2.c")
ulp_embed_binary(${ulp_app_name} "${ulp_lp_core_sources}" "${ulp_exp_dep_srcs}")
