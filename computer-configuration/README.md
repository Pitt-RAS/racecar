# Computer Configuration

This folder holds Ansible 'configuration as code' for configuring the on-board computers for Magellan. Currently there is only one computer, which has its configuration in `main-compute.yml`.

## SSH Access to the Robot

In order to get SSH access to the on-board computer, open a PR that appends your SSH public key to `authorized_keys` in this folder. Ask someone with access to apply the configuration after your PR is accepted.

## Provisioning a fresh robot computer

Assuming you've started with a fresh Ubuntu 18.04 install:

### On The Robot

Edit `main-compute.yml` changing `hosts: all` to `hosts: localhost`
```bash
sudo apt install ansible
ansible-playbook main-compute.yml
```

### From Your Laptop
Where `10.0.0.2` is the IP of the robot and `ubuntu` is the name of the user to SSH as.

```bash
sudo apt install ansible
ansible-playbook -i 192.168.1.22, -u ras -k -K main-compute.yml
```
