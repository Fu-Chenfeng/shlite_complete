# Connect a new laptop to the existing NUC

Do this once per laptop. Use a separate SSH key for each laptop. The procedure
adds an authorized login key; it does not start controllers or change calibration.

## 1. Network

Connect laptop USB → Ethernet adapter → Ethernet cable → NUC Ethernet port.
Power on the NUC. Find the new laptop's adapter and its connection name:

```bash
ip -br addr
nmcli device status
```

Use the actual connection name below. This gives the laptop `10.9.11.1/24`,
provides DHCP to the NUC and preserves Wi-Fi as the default route:

```bash
sudo nmcli connection modify "Wired connection 2" \
  ipv4.method shared ipv4.addresses 10.9.11.1/24 \
  ipv4.gateway "" ipv4.never-default yes
sudo nmcli connection up "Wired connection 2"
```

Do not attach the old workstation to this network while reusing its address.
Reconnect the NUC cable, wait for DHCP, and find its current address:

```bash
sudo apt install arp-scan
sudo arp-scan --interface=YOUR_USB_ETHERNET_INTERFACE --localnet
```

The current NUC-side adapter MAC is `00:0e:c6:30:e6:0c`; its last observed IP is
`10.9.11.186`. DHCP can change that address. The laptop-side adapter was
`enx000ec630ee60` on the original laptop; another laptop can use a different name.

## 2. Create the NEW laptop's key

On the **new laptop**:

```bash
ssh-keygen -t ed25519 -f ~/.ssh/shadow_nuc
cat ~/.ssh/shadow_nuc.pub
```

Do not overwrite an existing key. Choose a passphrase if desired. Copy the whole
public line beginning `ssh-ed25519`. Keep `~/.ssh/shadow_nuc` (the private key)
on this laptop; transfer only the `.pub` line to the old workstation.

## 3. Authorize it using the OLD workstation

Reconnect the NUC to a workstation that already has working authorized SSH
access. Discover its address there if necessary; the old setup expected
`10.9.11.2`. On that **old workstation**, substitute the NUC's actual address:

```bash
ssh user@NUC_ADDRESS hostname
ssh user@NUC_ADDRESS 'umask 077; mkdir -p ~/.ssh; cat >> ~/.ssh/authorized_keys'
```

Paste the new laptop's entire public-key line, press **Enter**, then **Ctrl+D**.
This appends the key and preserves existing access. Add it only once. If the
old workstation also asks for an unknown account password, use the account/key
that previously worked or ask the NUC administrator; copying Docker images
cannot grant SSH access. If you know the NUC account password, you can instead
run `ssh-copy-id -i ~/.ssh/shadow_nuc.pub user@NUC_ADDRESS` on the new laptop.

## 4. Test from the NEW laptop

Reconnect the NUC to the new laptop. Scan for its address again, then:

```bash
ssh -i ~/.ssh/shadow_nuc -o IdentitiesOnly=yes user@NUC_ADDRESS hostname
```

It should print `nuc-control`. Verify the SSH host key when prompted. An encrypted
private key may ask for its passphrase; that is not the NUC account password.
For repeated commands, optionally unlock it in your desktop SSH agent:

```bash
ssh-add ~/.ssh/shadow_nuc
```

## 5. Clock synchronization

The existing NUC expects its time server at `10.9.11.1`. On the new laptop:

```bash
sudo apt install chrony
```

Add this line once to `/etc/chrony/chrony.conf`:

```text
allow 10.9.11.0/24
```

Keep the laptop connected to its normal time sources, then:

```bash
sudo systemctl restart chrony
chronyc -n tracking
ssh -i ~/.ssh/shadow_nuc user@NUC_ADDRESS chronyc -n tracking
```

Wait until the NUC's `Reference ID` shows `10.9.11.1` and `Leap status` is
`Normal`. If needed, an authorized NUC administrator can restart Chrony there.
`./robotctl real nuc` checks this before starting the hand hardware loop.

## 6. Configure the project and UR5 network

```bash
cd ~/shlite_complete
./robotctl init
```

Edit `config.yaml` under `network`: set `pc_ip: 10.9.11.1`, the discovered
`nuc_ip`, `ssh_key` and `ur5_ip`. Review the host UID/GID and desktop display in
`.env`. All commands and desktop controls automatically read these files. No
private key belongs in the project. Network values left in an older `.env` are
superseded by `config.yaml`.

Use a separate Ethernet connection to the UR5. The working setup uses laptop
`192.168.1.20/24` and robot `192.168.1.10`; that connection needs no default
gateway. Keep the pendant's External Control program pointed at the laptop's
UR-network address. Set `network.ur5_reverse_ip: 192.168.1.20` in `config.yaml` if automatic detection is
unsuitable. Check both routes:

```bash
ip route get NUC_ADDRESS
ip route get 192.168.1.10
```

Then follow [README.md](../README.md). The existing NUC container should remain
named `dexterous_hand`; the project overrides numeric ROS addresses per launch,
so it does not depend on the old `nuc-control → 10.9.11.2` hosts entry.
