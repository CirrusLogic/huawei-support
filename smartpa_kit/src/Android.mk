# Copyright (C) 2020 Cirrus Logic, Inc. and
#                    Cirrus Logic International Semiconductor Ltd.
#                    All rights reserved.
#
# This software as well as any related documentation is furnished under
# license and may only be used or copied in accordance with the terms of the
# license.  The information in this file is furnished for informational use
# only, is subject to change without notice, and should not be construed as
# a commitment by Cirrus Logic.  Cirrus Logic assumes no responsibility or
# liability for any errors or inaccuracies that may appear in this file
# or any software that may be provided in association with this file.
#
# Except as permitted by such license, no part of this file may be
# reproduced, stored in a retrieval system, or transmitted in any form or by
# any means without the express written consent of Cirrus Logic.
#
# Warning
#    This software is specifically written for Cirrus Lgic devices.
#    It may not be used with other devices.

LOCAL_PATH:= $(call my-dir)


include $(CLEAR_VARS)
LOCAL_MODULE := libsmartpakit
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS += -Werror -Wno-error=unused-parameter -Wno-unused-parameter -Wall

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/include-all

LOCAL_SRC_FILES := \
	SmartPAKit.cpp

LOCAL_SHARED_LIBRARIES := \
	liblog \
	libutils

include $(BUILD_SHARED_LIBRARY)
