// ----------------------------------------------------------------------------
// Copyright 2016-2017 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "simplem2mclient.h"
#ifdef TARGET_LIKE_MBED
#include "mbed.h"
#endif
#include "GroveGPS.h"
#include "LM75B.h"

extern int GPS_init(void);
extern void gps_send(void);
extern GroveGPS gps;
LM75B sensor(D14, D15);
int LM75B_present = 0;;


static int main_application(void);


int main()
{
    // run_application() will first initialize the program and then call main_application()


    return run_application(&main_application);
}

// Pointers to the resources that will be created in main_application().
static M2MResource* button_res;
static M2MResource* pattern_res;
static M2MResource* blink_res;
static M2MResource* lattitude_res;
static M2MResource* longitude_res;
static M2MResource* gps_time_res;
static M2MResource* altitude_res;
static M2MResource* course_res;
static M2MResource* speed_res;
static M2MResource* uncertanty_res; 
static M2MResource* junk_res;
static M2MResource* temperature_res;
static M2MResource* location_res;


// Pointer to mbedClient, used for calling close function.
static SimpleM2MClient *client;

void pattern_updated(const char *)
 {
    printf("PUT received, new value: %s\n", pattern_res->get_value_string().c_str());
}

void blink_callback(void *) {
    String pattern_string = pattern_res->get_value_string();
    const char *pattern = pattern_string.c_str();
    printf("LED pattern = %s\n", pattern);
    // The pattern is something like 500:200:500, so parse that.
    // LED blinking is done while parsing.
    toggle_led();
    while (*pattern != '\0') {
        // Wait for requested time.
        do_wait(atoi(pattern));
        toggle_led();
        // Search for next value.
        pattern = strchr(pattern, ':');
        if(!pattern) {
            break; // while
        }
        pattern++;
    }
    led_off();
}

void generic_notification_status_callback(const M2MBase& object, const NoticationDeliveryStatus status)
{
    switch(status) {
        case NOTIFICATION_STATUS_BUILD_ERROR:
            printf("Notification callback: (%s) error when building CoAP message\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_RESEND_QUEUE_FULL:
            printf("Notification callback: (%s) CoAP resend queue full\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SENT:
            printf("Notification callback: (%s) Notification sent to server\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_DELIVERED:
            printf("Notification callback: (%s) Notification delivered\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SEND_FAILED:
            printf("Notification callback: (%s) Notification sending failed\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SUBSCRIBED:
            printf("Notification callback: (%s) subscribed\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_UNSUBSCRIBED:
            printf("Notification callback: (%s) subscription removed\n", object.uri_path());
            break;
        default:
            break;
    }
}

void update_gps_resources(void)
{
    char tempbuffer[40];
	char labuf[16], lobuf[16];
	int months[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	int tday, hours, minutes, seconds, tt;
	gps.getLatitude(labuf);
	lattitude_res->set_value( (const uint8_t *) labuf, strlen(labuf)); 
    gps.getLongitude(lobuf);
	longitude_res->set_value( (const uint8_t *) lobuf, strlen(lobuf)); 
    gps.getTimestamp(tempbuffer);
	gps_time_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer)); 
	gps.getUncertanty(tempbuffer);
	uncertanty_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer)); 
    gps.getAltitude(tempbuffer);
	altitude_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer));
	gps.getCourse(tempbuffer);
	course_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer));
	gps.getSpeed(tempbuffer);
	speed_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer));
	tt = gps.gps_zda.utc_time;
    hours = tt / 10000;
    tt = tt - (hours*10000);
    minutes = tt / 100;
    seconds = tt - (minutes*100);
	// Correct UTC to PAcific time
	hours -= 7;
	tday = gps.gps_zda.day;
		if(hours < 0) { // UTC Correct
			hours += 24;
			tday -= 1;
			if(!tday) {
				tday = months[gps.gps_zda.month];
				}
		}
	if( LM75B_present) {
		sprintf( tempbuffer, "%.3f", sensor.temp());
		temperature_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer)); 
	}

	if(gps.gps_gga.position_fix) {
		sprintf( tempbuffer, "{\"name\":\"Jim\", \"lat\":\"%s\", \"lon\":\"%s\"}", labuf, lobuf);

		location_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer));
		sprintf(tempbuffer, "%d/%d/%d %d:%02d:%02d", gps.gps_zda.month, tday, gps.gps_zda.year, hours, minutes, seconds);
		junk_res->set_value( (const uint8_t *) tempbuffer, strlen(tempbuffer)); 
	}
}


// This function is called when a POST request is received for resource 5000/0/1.
void unregister(void *)
{
    printf("Unregister resource executed\n");
    client->close();
}

// This function is called when a POST request is received for resource 5000/0/2.
void factory_reset(void *)
{
    printf("Factory reset resource executed\n");
    client->close();
    kcm_status_e kcm_status = kcm_factory_reset();
    if (kcm_status != KCM_STATUS_SUCCESS) {
        printf("Failed to do factory reset - %d\n", kcm_status);
    } else {
        printf("Factory reset completed. Now restart the device\n");
    }
}

int main_application(void)
{
    // IOTMORF-1712: DAPLINK starts the previous application during flashing a new binary
    // This is workaround to prevent possible deletion of credentials or storage corruption
    // while replacing the application binary.
#ifdef TARGET_LIKE_MBED
    wait(2);
#endif


    // SimpleClient is used for registering and unregistering resources to a server.
    SimpleM2MClient mbedClient;
	GPS_init();
	if (sensor.open()) {
        printf("LM75B detected!\n");
		LM75B_present = 1;
        } else {
            error("LM75B not detected!\n");
        }
    // Save pointer to mbedClient so that other functions can access it.
    client = &mbedClient;

#ifdef MBED_HEAP_STATS_ENABLED
    printf("Client initialized\r\n");
    heap_stats();
#endif

    // Create resource for lattitude. Path of this resource will be: 3336/0/5514.
    lattitude_res = mbedClient.add_cloud_resource(3336, 0, 5514, "lattitude_resource", M2MResourceInstance::STRING,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);
							  
    longitude_res = mbedClient.add_cloud_resource(3336, 0, 5515, "longitude_resource", M2MResourceInstance::STRING,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);	
							  
    location_res = mbedClient.add_cloud_resource(3336, 0, 5597, "location_resource", M2MResourceInstance::STRING,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);	
							  							  
    uncertanty_res = mbedClient.add_cloud_resource(3336, 0, 5516, "Uncertanty_resource", M2MResourceInstance::STRING,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);	
							  							  
    gps_time_res = mbedClient.add_cloud_resource(3336, 0, 5518, "GPStime_resource", M2MResourceInstance::TIME,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);

    altitude_res = mbedClient.add_cloud_resource(3336, 0, 5799, "Altitude_resource", M2MResourceInstance::STRING,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);

	course_res = mbedClient.add_cloud_resource(3336, 0, 5705, "Course_resource", M2MResourceInstance::STRING,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);

	speed_res = mbedClient.add_cloud_resource(3336, 0, 5517, "Speed_resource", M2MResourceInstance::STRING,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);

	junk_res = mbedClient.add_cloud_resource(3336, 0, 5598, "Date-Time_resource", M2MResourceInstance::STRING,
                             M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);

	if(LM75B_present) {
		temperature_res = mbedClient.add_cloud_resource(3303, 0, 5700, "Temperature_resource", M2MResourceInstance::FLOAT,
							M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);
	}
	
    // Create resource for button count. Path of this resource will be: 3200/0/5501.
    button_res = mbedClient.add_cloud_resource(3200, 0, 5501, "button_resource", M2MResourceInstance::INTEGER,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)generic_notification_status_callback);

    // Create resource for led blinking pattern. Path of this resource will be: 3201/0/5853.
    pattern_res = mbedClient.add_cloud_resource(3201, 0, 5853, "pattern_resource", M2MResourceInstance::STRING,
                               M2MBase::GET_PUT_ALLOWED, "500:500:500:500", false, (void*)pattern_updated, NULL);

    // Create resource for starting the led blinking. Path of this resource will be: 3201/0/5850.
    blink_res = mbedClient.add_cloud_resource(3201, 0, 5850, "blink_resource", M2MResourceInstance::STRING,
                             M2MBase::POST_ALLOWED, "", false, (void*)blink_callback, NULL);

    // Create resource for unregistering the device. Path of this resource will be: 5000/0/1.
    mbedClient.add_cloud_resource(5000, 0, 1, "unregister", M2MResourceInstance::STRING,
                 M2MBase::POST_ALLOWED, NULL, false, (void*)unregister, NULL);

    // Create resource for running factory reset for the device. Path of this resource will be: 5000/0/2.
    mbedClient.add_cloud_resource(5000, 0, 2, "factory_reset", M2MResourceInstance::STRING,
                 M2MBase::POST_ALLOWED, NULL, false, (void*)factory_reset, NULL);

    // Print to screen if available.
    clear_screen();
    print_to_screen(0, 3, "Cloud Client: Connecting");

    mbedClient.register_and_connect();

    // Check if client is registering or registered, if true sleep and repeat.
    while (mbedClient.is_register_called()) {
        static int button_count = 0;
        do_wait(1000);

		update_gps_resources();
		
        if (button_clicked()) {
            button_res->set_value(++button_count);
        }
    }

    // Client unregistered, exit program.
    return 0;
}
