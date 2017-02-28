/*
 * Tegra capture IVC driver.
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAPTURE_IVC_H
#define INCLUDE_CAPTURE_IVC_H

/*
 * Enqueue the control message binary blob to ivc driver's control channel
 * queue, which is to be transfered over control IVC channel to RTCPU.
 *
 * control_desc: Binary blob containing control message descriptor,
 * is opaque to KMDs.
 */
int tegra_capture_ivc_control_enqueue(void *control_desc);

/*
 * Enqueue the capture message binary blob to ivc driver's capture channel
 * queue, which is to be transfered over capture IVC channel to RTCPU.
 *
 * capture_desc_blob: Binary blob containing capture message descriptor,
 * is opaque to KMDs.
 */
int tegra_capture_ivc_capture_enqueue(void *capture_desc);

/*
 * Callback function to be registered by client to receive the rtcpu
 * notifications through control or capture ivc channel.
 *
 * rtcpu_resp: Binary blob containing the response message received from
 * rtcpu through control or capture ivc channel, its opaque to KMDs.
 */
void (*tegra_capture_ivc_cb_func)(void *rtcpu_resp);

/*
 * Register callback function to receive response messages from rtcpu
 * through control ivc channel.
 *
 * control_resp_cb: callback function to be registered for control ivc channel.
 */
void tegra_capture_ivc_register_control_cb(
		tegra_capture_ivc_cb_func control_resp_cb);

/*
 * Register callback function to receive status-indication messages from
 * rtcpu through capture ivc channel.
 *
 * capture_status_ind_cb: callback function to be registered for capture ivc
 * channel.
 */
void tegra_capture_ivc_register_capture_cb(
		tegra_capture_ivc_cb_func capture_status_ind_cb);

#endif /* INCLUDE_CAPTURE_IVC_H */
