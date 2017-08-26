FROM heichblatt/archlinux-yaourt

MAINTAINER Patrick José Pereira <patrickelectric@gmail.com>

# Update pacman
RUN sudo pacman-key --populate && \
    sudo pacman -Syyu --noconfirm

# Install some packages
RUN sudo pacman -S --noconfirm \
    base-devel \
    binutils \
    make \
    git \
    gcc \
    clang \
    cmake \
    && (echo -e "y\ny\n" | sudo pacman -Scc)

# Get emlid rtklib
RUN cd ~ && \
    sudo git clone -b reach https://github.com/emlid/RTKLIB && \
    cd RTKLIB/app && \
    sudo sh makeall.sh && \
    echo "Finding some executables..." && \
    sudo find . -type f -executable -print
#RUN

#Configure user, git and kf5
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
#COPY gitconfig /tmp/.gitconfig
RUN sudo chmod 755 /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

#RUN cd ~
#RUN cd RTKLIB
#RUN ls

