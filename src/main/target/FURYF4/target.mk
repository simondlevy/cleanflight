F405_TARGETS    += $(TARGET)
ifeq ($(TARGET), FURYF4OSD)
FEATURES        += VCP ONBOARDFLASH
else
FEATURES        += VCP ONBOARDFLASH SDCARD
endif

TARGET_SRC = drivers/accgyro/accgyro_spi_mpu6000.c 
