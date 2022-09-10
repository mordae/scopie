idf_component_register(
	SRCS
		"main.c"
		"fontx.c"
		"ili9340.c"

	INCLUDE_DIRS
		"."

	PRIV_REQUIRES
		driver
		esp_adc
		nvs_flash
		soc
		spiffs
		vfs
)
