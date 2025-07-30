FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive



RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pip \
      libgl1-mesa-dri \
      libgl1-mesa-glx \
      wget \
      ca-certificates \
      # utility di debug GPU (opzionali ma utili)
      clinfo \
      rocminfo \
      pulseaudio-utils \
    && rm -rf /var/lib/apt/lists/*


RUN pip install openvr

RUN pip install visual-kinematics

RUN pip install roboticstoolbox-python spatialmath-python scipy

  
  
RUN useradd -m steam
    
USER root

#COPY entrypoint.sh /entrypoint.sh
#RUN chmod +x /entrypoint.sh
#ENTRYPOINT ["/entrypoint.sh"]
 
 
