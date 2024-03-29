/*
 * reuc_pitl.c
 *
 * This code runs the REUC controller on the target flight computer, using UDP to
 * get inputs from the simulated sensors and to send commands back.
 *
 * Build with:
 *
 * Linux/macOS: gcc -o linux/reuc_pitl reuc.c reuc_pitl.c ../utilities/udp-helper/udp-helper.c -lm
 *
 */

#include <stdio.h>
#include <stdint.h>
#include "../utilities/udp-helper/udp-helper.h"

#include "reuc.h"

int main(void)
{
    /* UDP communication stuff */
    UDPHelper comms;
    const char * target_ip_address = "127.0.0.1"; /* Dummy; we'll respond to whomever talks to us first. */
    const int listen_port = 2000;
    const int target_port = 2001;
    const int respond_to_sender = 1;

    /* Inputs and outputs */
    uint64_t status = 0;
    double I = 0.;
    double kappa_c = 0.;
    double mu_c = 0.;
    double rho = 0.;
    double alpha = 0.;
    double q_TI[4]  = {0., 0., 0., 0.};
    double q_BI[4]  = {0., 0., 0., 0.};
    double w_BI_B[3]  = {0., 0., 0.};
    double tau_B[3]  = {0., 0., 0.};

    /* Bookkeeping */
    int bytes_received = 0;
    int bytes_sent = 0;

    /* Create a socket and listen for incoming UDP messages. */
    printf("Running REUC-PITL.\n");
    udp_helper(&comms, listen_port, target_port, respond_to_sender, target_ip_address);

    /* Loop until the first 8 bytes of the message tell us not to any more. */
    while (1)
    {
        /* Pull the data in. */
        //printf("Waiting for new data.\n");
        bytes_received = udp_receive(&comms);
        //printf("Received %d bytes.\n", bytes_received);
        udp_pull(&comms, &status, sizeof(status));
        if (status <= 0) { break; } /* See if we should quit. */
        udp_pull(&comms, &I,       sizeof(I));
        udp_pull(&comms, &kappa_c, sizeof(kappa_c));
        udp_pull(&comms, &mu_c,    sizeof(mu_c));
        udp_pull(&comms, &rho,     sizeof(rho));
        udp_pull(&comms, &alpha,   sizeof(alpha));
        udp_pull(&comms, q_TI,     sizeof(q_TI));
        udp_pull(&comms, q_BI,     sizeof(q_BI));
        udp_pull(&comms, w_BI_B,   sizeof(w_BI_B));
        //printf("w1=%5.2f, w2=%5.2f, w3=%5.2f\n", w_BI_B[0], w_BI_B[1], w_BI_B[2]);

        /* Run the target code. */
        reuc(I, kappa_c, mu_c, rho, alpha, q_TI, q_BI, w_BI_B, tau_B);

        /* Push the data back out. */
        udp_push(&comms, tau_B, sizeof(tau_B));
        bytes_sent = udp_transmit(&comms);
        //printf("Sent %d bytes.\n", bytes_sent);
    }

    /* Tidy up. */
    udp_close(&comms);
    printf("Done.\n");
    return 0;
}
