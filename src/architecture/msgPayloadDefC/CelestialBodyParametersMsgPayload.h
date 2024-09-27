/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef CELESTIAL_BODY_PARAMETERS
#define CELESTIAL_BODY_PARAMETERS

#define MAX_STRING_LENGTH 256
#define MAX_PARAMETER_LENGTH 12

/*! @brief Celestial body parameters message*/

typedef struct {
    char bodyName[MAX_STRING_LENGTH];
    char shapeModel[MAX_STRING_LENGTH];
    double perlinNoise;
    double proceduralRocks;
    char brdf[MAX_STRING_LENGTH];
    double reflectanceParameters[MAX_PARAMETER_LENGTH];
    double meanRadius;
    double principalAxisDistortion;
}CelestialBodyParametersMsgPayload;

#endif //CELESTIAL_BODY_PARAMETERS
