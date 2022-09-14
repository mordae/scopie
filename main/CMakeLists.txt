idf_component_register(
	SRCS
		"main.c"
		"ili9225.c"
		"rotary.c"

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
