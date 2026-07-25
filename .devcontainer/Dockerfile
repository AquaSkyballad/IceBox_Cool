ARG DOCKER_TAG=v5.5.5
FROM espressif/idf:${DOCKER_TAG}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

# 如果基础镜像已经有同名 UID/GID 的用户/组（比如 Ubuntu 24.04 自带的 ubuntu 用户），
# 就把它删掉再建，避免冲突；这样不管基础镜像里叫什么名字都能处理。
RUN if getent passwd $USER_UID > /dev/null; then \
        EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1); \
        userdel -r $EXISTING_USER 2>/dev/null || userdel $EXISTING_USER; \
    fi \
    && if getent group $USER_GID > /dev/null; then \
        EXISTING_GROUP=$(getent group $USER_GID | cut -d: -f1); \
        groupdel $EXISTING_GROUP 2>/dev/null || true; \
    fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update -y && apt-get install -y udev clangd

RUN chown -R $USERNAME:$USERNAME /opt/esp

RUN usermod -aG dialout $USERNAME

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

USER $USERNAME

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash", "-c"]