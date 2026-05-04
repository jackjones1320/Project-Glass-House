idf_component_register(
    SRCS
        "main.c"
        "csi_collector.c"
        "stream_sender.c"
        "nvs_config.c"
        "edge_processing.c"
        "power_mgmt.c"
        "heartbeat.c"
        "link_reporter.c"
        "telemetry_sender.c"
        "send_watchdog.c"
        "crash_counter.c"
    INCLUDE_DIRS "."
)
