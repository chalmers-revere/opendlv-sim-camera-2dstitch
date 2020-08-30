# Copyright (C) 2020 Ola Benderius
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

FROM alpine:3.11 as builder
RUN apk update && \
    apk --no-cache add \
        glm-dev \
        g++ \
        make \
        cmake

ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install


FROM alpine:3.11
RUN apk update && \
    apk --no-cache add \
        glm

WORKDIR /usr/bin
COPY --from=builder /tmp/dest/ /usr
ENTRYPOINT ["/usr/bin/opendlv-sim-camera-2dstitch"]


