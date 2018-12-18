#!/bin/bash
#
#   This script formats and writes a SD card from an skylark provided tar ball.
#   Use with caution and select the path to the device node carefully.
#   Usage: sudo ./flash_sd_card.sh -t path/to/iris030_rootfs_2018-version.tar.gz
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#   DEALINGS IN THE SOFTWARE.
#
#   (c) info@skylarkwireless.com 2016
#   SPDX-License-Identifier: BSD-3-Clause

TMPDIR=/tmp

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Don't touch any drive that is larger than then below size. This is
# a safety feature to keep you from doing bad things to your drives.
MAX_BLK_SIZE=$((1024 * 1024 * 1024 * 16)) #8GiB - protection level
ISO_BLK_SIZE=$((1024 * 1024 * 1024 * 2 )) #2GiB - sets size of image

. ${THIS_DIR}/banner #add pw functionality
pw $PW_OK "$self: iris flash image creation tool and sd card writer"

BLOCK_SZ=4096
FoundDrive=false
IMG=false
MKFS_EXT4_ARGS=""
MKFS_VFAT_ARGS=""
RSYNC_ARGS=""
MNT_BOOT=""
MNT_ROOT=""
MNT_BOOT_ARGS="" #"-o sync,discard,flush"
MNT_ROOT_ARGS="" #"-o sync,discard,max_batch_time=10000"
PROJ_BASE=""
SUPER_BALLER=""
SWAP_SZ=256M
TGT_DEV=""
TGT_DEV_BOOT=""
TGT_DEV_ROOT=""
TAR_FILE=""
TAR_DIR=""
BOOT_BIN=""
FORMAT_ONLY=""
NO_CUSTOM=""

self=$(basename $0)
usage() {
   echo ""
   echo "$self:"
   echo "   wrapper to flash SD cards using tarball release"
   echo ""
   echo "Usage:"
   echo "   $self [-hvi:d:] <-t image.tar.gz>"
   echo ""
   echo "Options:"
   echo "   -h, --help          - display usage"
   echo "   -v, --verbose       - more info"
   echo "   -a, --auto          - find sd card and use it"
   echo "   -g, --gset          - set up nautilus to quit automounting"
   echo "   -d, --dev    <dev>  - use block device dev"
   echo "   -D           <dev>  - use block device dev without SD checks"
   echo "   -i, --img    <size> - writes to .img.bz2 (e.g., for etcher)"
   echo "   -s, --swap   <size> - sets swap file size"
   echo "   -t, --tar    <file> - use image from tarball"
   echo "   -T, --tar_dir<dir>  - use image from preextracted tarball (for vomit_sd) "
   echo "   -u, --update <file> - use BOOT.BIN"
   echo "   -y, --force-write   - do not prompt for confirmation! (dangerous)"
   echo ""
}

################################################################################
[[ $SKLK_NIGHTLY != 1 ]] && RSYNC_ARGS="--info=progress2" #show progress interactively
safe_cp () { #globs should be passed in as a string to be expanded here
    local src=$1
    local dst=$2
    shift 2
    local opts=$*
    local dst_dir=$(dirname $dst)
    local rc=0

    if [ ! -d $dst_dir ]; then
        mkdir -p $dst_dir
        rc=$?
        if [ $? -ne 0 ]; then
            pw $PW_ERR "failed to created necessary target dir:$dst_dir"
            cleanup
            exit $rc
        else
            pw $PW_NFO "created missing target dir: $dst_dir"
        fi
    fi

    pw $PW_DBG "Copying $src to $dst"
    pw $PW_DBG "opts=$*"
    rsync $RSYNC_ARGS $opts $src $dst
    rc=$?

    if [ $rc -ne 0 ]; then
        pw $PW_ERR "copy fail ($rc) $src to $dst"
        cleanup
        exit $rc
    else
        pw $PW_DBG "Success copying $src to $dst"
    fi
}


################################################################################
cleanup() {
    pw $PW_NFO "Syncing cache... This may take a while."
    pw $PW_DBG "$(lsblk | grep sd -B1)"
    pw $PW_NFO "During sync, \`watch grep -e Dirty: -e Writeback: /proc/meminfo' to watch progress."

    if [ -d "$MNT_BOOT" ]; then
        pw $PW_NFO "Syncing $MNT_BOOT"
        sync -f $MNT_BOOT
        pw $PW_DBG "sync=$?"
        pw $PW_NFO "Unmounting $TGT_DEV_BOOT"
        umount -f $TGT_DEV_BOOT
        pw $PW_DBG "umount=$?"
        rmdir $MNT_BOOT
    fi

    if [ -d "$MNT_ROOT" ]; then
        pw $PW_NFO "Syncing $MNT_ROOT"
        sync -f $MNT_ROOT
        pw $PW_DBG "sync=$?"
        pw $PW_NFO "Unmounting $TGT_DEV_ROOT"
        umount -f $TGT_DEV_ROOT
        pw $PW_DBG "umount=$?"
        rmdir $MNT_ROOT
    fi

    if [[ -d $TAR_DIR && ! -z $TAR_FILE ]]; then
        pw $PW_DBG "removing $TAR_DIR because it was self-extracted and not needed anymore"
        rm -rf $TAR_DIR
    fi

    if [ "$IMG" = true ]; then
        pw $PW_NFO "Removing image loop device $TGT_DEV"
        losetup -d $TGT_DEV
        #rm $IMG_FILE
    fi

    [ $# -ne 0 ] && exit $1 #someone wants exit immediately
}

ctrl_c() {
    pw $PW_WRN "Aborting"
    cleanup
}

trap cleanup SIGINT  #call cleanup
trap cleanup SIGTERM #call cleanup
################################################################################
# Arg Parsing
SOPTS="hfyagvd:s:D:i:t:T:u:"
LOPTS="help,format-only,force-write,auto,gset,verbose,dev:,img:,swap:,tar:,tar_dir:,update:"
OPTS=$(getopt -s bash --options $SOPTS --longoptions $LOPTS --name $self -- "$@")
if [ $? -ne 0 ]; then
    pw $PW_ERR "error in parsing arguments"
    exit -1
fi

eval set -- "$OPTS"
while :; do
    case $1 in
    -h|--help)
        usage
        exit 0
        ;;
    -s|--swap)
        SWAP_SZ=$1
        pw $PW_NFO "setting swap size to $SWAP_SZ"
        ;;
    -y|--force-write)
        pw $PW_WRN "bypassing safety prompts, baller"
        ForceSDWrite=true
        MKFS_EXT4_ARGS="-F -F"
        MKFS_VFAT_ARGS=""
        ;;
    -a|--auto)
        pw $PW_NFO "running auto scan for sd card"
        SD_AUTO=1
        ;;
    -v|--verbose)
        dbg=1
        pw $PW_DBG "enabling debug messages"
        ;;
    -D|-d|--dev)
        if [[ $1 == "-D" ]]; then
            pw $PW_WRN "bypassing SD checks on block device, super baller"
            SUPER_BALLER=1
        fi
        shift
        TGT_DEV=$1
        if [[ -b $TGT_DEV ]]; then
            pw $PW_NFO "block device: $TGT_DEV"
        else
            pw $PW_ERR "$TGT_DEV is not a block device"
            exit
        fi
        ;;
    -i|--img)
        shift
        SUPER_BALLER=1
        ForceSDWrite=true
        IMG_SIZE=$1
        IMG=true
        pw "creating compressed image of size $IMG_SIZE rather than writing to sd card"
        ;;
    -f|--format-only)
        FORMAT_ONLY=1
        pw $PW_NFO "Disk preparation only"
        ;;
    -u|--update)
        shift
        BOOT_BIN=$1
        if [ -z $BOOT_BIN ]; then
            pw $PW_ERR "filename is empty"
            exit -1
        fi

        if [ -e $BOOT_BIN ]; then
            pw $PW_NFO "update image: $BOOT_BIN"
        else
            pw "$PW_ERR $BOOT_BIN does not exist"
            exit -1
        fi
        ;;
    -T|--tar_dir)
        shift
        TAR_DIR=$1
        if [ -z $TAR_DIR ]; then
            pw $PW_ERR "dirname is empty"
            exit -1
        fi

        if [ -d $TAR_DIR ]; then
            pw $PW_NFO "tar directory: $TAR_DIR"
        fi
        ;;
    -t|--tar)
        shift
        TAR_FILE=$1
        if [ -z $TAR_FILE ]; then
            pw $PW_ERR "filename is empty"
            exit -1
        fi

        if [ -e $TAR_FILE ]; then
            pw $PW_NFO "tar target: $TAR_FILE"
        fi
        ;;
    -g|--gset)
        # The automatic launching of Nautilus is really, really annoying when
        # when drives are being mounted and unmounted. This disables it for Ubuntu.
        sudo gsettings set org.gnome.desktop.media-handling automount-open false
        [ $? -eq 0 ] && pw $PW_NFO "nautilus settings saved successfully"\
                     || pw $PW_ERR "failed to set settings"
        exit 0
        ;;
    --)
        shift
        break
        ;;
    *)
        shift
        break
        ;;
    esac
    shift
done

# Check input arguments
# checks to make sure all requsite files/dirs can be found
[[ -z $TAR_FILE && -z $TAR_DIR && -z $BOOT_BIN ]] && pw $PW_ERR "required tar image not found. use -t or -T to specify source, or -u for update" && error=1
[[ ! -z $error ]] && exit -$error

if [ $EUID -ne 0 ]; then
    pw $PW_ERR "this script is required to be run as root user as it modifies partition tables and accesses block devices directly"
    exit -1
fi


SKLK_PROJ_NAME="${PROJ_BASE}_auto"
SKLK_HW_NAME="${PROJ_BASE}_top"

create_lo_dev() {
    local img_file=$1
    local img_size=$2
    echo "[SKLK] Creating raw image file $img_file of size $img_size"
    truncate -s $img_size $img_file
    local rc=$?;
    if [ $rc -ne 0 ]; then
        pw $PW_ERR "creating image file $img_file of size $img_size failed!"
        exit $rc
    fi
    ldev=`losetup -f`
    local rc=$?;
    if [ $rc -ne 0 ]; then
        pw $PW_ERR "Cannot allocate loopback device."
        exit $rc
    fi
    losetup -P $ldev $img_file #-P is important, it enables partition scans
}

partition_dev() {
    local dev=$1

    # Format and re-partition into boot and filesystem partitions
    echo "[SKLK] Creating new partitions on: $dev"

    echo "Forcing deletion of all partitions..."
    #Forcefully wipe every partition.  This avoids a prompt in fdisk
    #as well as cleans up any remaining partition names.
    for p in `ls ${1}*`
    do
        if [ $p != ${1} ]; then
            if [ -b $p ]; then #should be unnecessary, but just in case...
                dd if=/dev/zero of=$p bs=8192 count=8 #nuclear
            fi
            #wipefs $p wtf?  this doesn't seem to work
        fi
    done

    # http://superuser.com/questions/332252/creating-and-formating-a-partition-using-a-bash-script
    # to create the partitions programatically (rather than manually)
    # we're going to simulate the manual input to fdisk
    # The sed script strips off all the comments so that we can
    # document what we're doing in-line with the actual commands
    # Note that a blank line (commented as "default" will send a empty
    # line terminated with a newline to take the fdisk default.
    # Note: you can't have leading white space on the EOF line,
    #       hence the crappy formatting.
    sed -e 's/\s*\([\+0-9a-zA-Z]*\).*/\1/' << EOF | fdisk $dev
o # clear the in memory partition table
n # new partition
p # primary partition
1 # partition number 1
# default - start at beginning of disk
+512M # 512 MB boot partion
t # set partition type
#default
b # vfat
n # new partition
p # primary partition
2 # partition number 2
# default - start at beginning of disk
+1G # 1 GB swap partion
t # set partition type
#default
82 # linux swap
n # new partition
p # primary partition
3 # partion number 3
# default, start immediately after preceding partition
# default, extend partition to end of disk
a # make a partition bootable
1 # bootable partition is partition 1 -- /dev/sda1
p # print the in-memory partition table
w # write the partition table
q # and we're done
EOF
    local rc=$?; #fdisk
    if [ $rc -ne 0 ]; then
        pw $PW_ERR "fdisk $dev fail($rc)"
        exit $rc
    else
        pw $PW_NFO "fdisk completed succesfully"
    fi

}
partition_wait() {
    local dev=$1
    local timeout=$2

    while [[ $timeout -ne 0 ]]; do
        num_devs=$(ls ${dev}* | wc -l)
        [ $num_devs -eq 4 ] && break
        sleep 1
        timeout=$((timeout - 1))
    done
    if [ $timeout -eq 0 ]; then
        pw $PW_ERR "timeout waiting for partitions"
        cleanup -1
    fi
}

partition_load() {
    local dev=$1
    local retry=$2
    local retry_gap=$3

    while [[ $retry -ne 0 ]]; do
        sleep $retry_gap
        hdparm -z $TGT_DEV
        local rc=$?;
        if [ $rc -ne 0 ]; then
           retry=$((retry - 1))
           pw $PW_WRN "hdparm -z $TGT_DEV fail($rc), retrying $retry more times"
           continue
        fi
        pw $PW_NFO "partitions loaded successfully"
        break
    done

    if [ $retry -eq 0 ]; then
        pw $PW_ERR "out of retries waiting for partitions"
        exit $rc
    fi

}

check_partitions(){
    local dev=$1
    # Create the vfat and ext4 file systems on the new partitions
    # MMC card partitions get a different suffix.
    pw $PW_DBG "Checking for block device naming of ${dev}"
    if [ -b "${dev}p1" ] ; then
        TGT_DEV_BOOT="${dev}p1"
        TGT_DEV_SWAP="${dev}p2"
        TGT_DEV_ROOT="${dev}p3"
    elif [ -b "${TGT_DEV}1" ] ; then
        TGT_DEV_BOOT="${dev}1"
        TGT_DEV_SWAP="${dev}2"
        TGT_DEV_ROOT="${dev}3"
    else
        ls -la ${dev}*
        pw $PW_ERR "Partitions created by fdisk not detected!"
        exit -1
    fi

    pw $PW_DBG "File system vfat (boot partition): $TGT_DEV_BOOT"
    pw $PW_DBG "File system swap: $TGT_DEV_SWAP"
    pw $PW_DBG "File system ext4 (linaro fs): $TGT_DEV_ROOT"
}

format_dev(){
    local dev=$1
    local rc=0

    check_partitions $dev

    pw $PW_DBG "Formatting vfat (boot partition): $TGT_DEV_BOOT"
    mkfs.vfat $MKFS_VFAT_ARGS -n ZED_BOOT $TGT_DEV_BOOT
    rc=$?; [ $rc -ne 0 ] && pw $PW_ERR "failed to mkfs($rc)" && exit $rc
    sync -f $TGT_DEV_BOOT

    pw $PW_DBG "Formatting swap: $TGT_DEV_SWAP"
    mkswap $TGT_DEV_SWAP
    rc=$?; [ $rc -ne 0 ] && pw $PW_ERR "failed to mkswap($rc)" && exit $rc
    sync -f $TGT_DEV_SWAP

    pw $PW_DBG "Formatting ext4 (linaro fs): $TGT_DEV_ROOT"
    mkfs.ext4 $MKFS_EXT4_ARGS -L ROOT_FS $TGT_DEV_ROOT
    rc=$?; [ $rc -ne 0 ] && pw $PW_ERR "failed to mkfs($rc)" && exit $rc
    sync -f $TGT_DEV_ROOT

    pw $PW_NFO "Formatted $dev successfully"
}

mount_dev() {
    local rc=0
    # Mount the new partitions
    pw $PW_DBG "Re-mounting new formatted drives..."

    MNT_BOOT=$(mktemp --tmpdir -d ZED_BOOT.XXXX)
    mount $MNT_BOOT_ARGS "$TGT_DEV_BOOT" $MNT_BOOT
    rc=$?; [ $rc -ne 0 ] && pw $PW_ERR "failed to mount($rc) $MNT_BOOT" && exit $rc

    MNT_ROOT=$(mktemp --tmpdir -d ZED_ROOT.XXXX)
    mount $MNT_ROOT_ARGS "$TGT_DEV_ROOT" $MNT_ROOT
    ac=$?; [ $rc -ne 0 ] && pw $PW_ERR "failed to mount($rc) $MNT_ROOT" && exit $rc

    pw $PW_NFO "Mounted $dev successfully"
}

check_blk_sz(){
    local dev=$1
    # Test the size of the device--this is an additional safety step!
    dsize=$(blockdev --getsize64 ${dev})
    if [[ "$dsize" -gt "$MAX_BLK_SIZE" ]] ; then
        pw $PW_WRN "Drive $dsize is larger than current limit $MAX_BLK_SIZE !"
        return -1
    fi
    pw $PW_DBG "check_blk_sz passed for ${dev}"
    return 0
}

check_device() {
    local dev=$1
    pw $PW_DBG "Checking: ${dev}"
    IsFlashable=0
    IsFlash=$( udevadm info --query=all --name=${dev} | grep ID_DRIVE_FLASH )
    IsSDCard=$(udevadm info --query=all --name=${dev} | grep SD_Card        )
    IsMMCSD=$( udevadm info --query=all --name=${dev} | grep SD_MMC         )
    IsThumb=$( udevadm info --query=all --name=${dev} | grep ID_DRIVE_THUMB )
    pw $PW_DBG "
IsFlash  = $IsFlash
IsSDCard = $IsSDCard
IsMMCSD  = $IsMMCSD
IsThumb  = $IsThumb"

    # Identified to udisk in VirtualBox with Transcend SD Reader
    # Identified to udisk in VirtualBox with Microsoft Surface Driver
    if [ -n "$IsThumb" ] ; then
        IsFlashable=0
    elif [ -n "$IsFlash" ] || [ -n "$IsSDCard" ] || [ -n "$IsMMCSD" ] ; then
        check_blk_sz $dev
        [ $? -eq 0 ] && IsFlashable=1 || IsFlashable=0
    fi

    if [ $IsFlashable -eq 1 ] ; then
        lsblk $dev && return $?
    fi
    return -1
}

find_sd_card() {
    # Try to detect and flash an SD card, with a number of safeguards to not accidentially
    # overwrite your OS filesystem ;)
    pw $PW_NFO "Searching for a valid SD card..."
    #sudo apt-get install procinfo lshw
    blockdevs=`lsblk | grep disk | awk '{print $1}'`
    pw $PW_DBG "blockdevs:
$blockdevs"
    if [ ! -z $TGT_DEV ]; then
        if [ ! -z $SUPER_BALLER ]; then
            FoundDrive=true
            pw $PW_NFO "Found SD card $TGT_DEV with:"
            lsblk $TGT_DEV
            return $?
        else
            check_device $TGT_DEV
            if [ $? -eq 0 ]; then
                FoundDrive=true
                pw $PW_NFO "Found SD card $TGT_DEV with:"
                lsblk $TGT_DEV
                return $?
            fi
        fi
    else for dname in $blockdevs; do
        check_device "/dev/$dname"
        if [ $? -eq 0 ]; then
            FoundDrive=true
            TGT_DEV="/dev/$dname"
            pw $PW_NFO "Found SD card $TGT_DEV with:"
            lsblk $TGT_DEV
            return $?
        fi
        done
    fi
    return -1

}

create_swap_file () {
    local sz=$1
    local froot=$2
    local fstab=$froot/etc/fstab
    local fname=/media/swapfile
    local fswap=$froot/$fname

    pw $PW_NFO "Creating swapfile of size $sz in $froot"

    fallocate -l $sz $fswap
    [ $? -ne 0 ] && return -1

    chmod 0600 $fswap
    [ $? -ne 0 ] && return -1

    mkswap $fswap
    [ $? -ne 0 ] && return -1

    echo "$fname none swap sw 0 0" >> $fstab
    [ $? -ne 0 ] && return -1

    return 0
}

# If we're writing to an image file then we setup it up and mount it as a loop
# device, then set the target device appropriately
if [ "$IMG" = true ]; then
    TAR_FILENAME=$(basename -- "$TAR_FILE")
    IMG_FILE=$TMPDIR/${TAR_FILENAME%.*.*}.img
    create_lo_dev $IMG_FILE $IMG_SIZE
    TGT_DEV=$ldev
fi

find_sd_card
[ $? -ne 0 ] && pw $PW_ERR "find_sd_card failed" && exit -1
pw $PW_DBG "find_sd_card: TGT_DEV=$TGT_DEV"

# Ask for user confirmation
if [ "$ForceSDWrite" = true ] ; then
    UserAccepts=true
else
    UserAccepts=false
    if [ -n "$TGT_DEV" ] && [ ! "$ForceSDWrite" = true ]; then
        while true; do
            read -p "[SKLK] Do you wish to flash $TGT_DEV? " yn
            case $yn in
                [Yy]* ) UserAccepts=true; break;;
                [Nn]* ) break;;
                * ) echo "Please answer with: [YyNn].";;
            esac
        done
    fi # Get user input
fi # Do not prompt for input

# update mode (mount, copy BOOT.BIN, cleanup)
if [ -n "$TGT_DEV" ] && [ "$UserAccepts" = true ] && [ ! -z $BOOT_BIN ]; then
    check_partitions ${TGT_DEV}
    mount_dev      ${TGT_DEV}
    pw $PW_NFO "Copying $BOOT_BIN to $TGT_DEV"
    safe_cp $BOOT_BIN $MNT_BOOT/ -vrc

    ############################################################################
    pw $PW_OK "$self: Update completed!"
    cleanup #sync cache and unmount
    eject $TGT_DEV
    pw $PW_NFO "SD card $TGT_DEV is updated with $BOOT_BIN. It is safe to remove it."
    pw $PW_OK "$self: Done!"

# If a drive was found, partition it, format it, and copy over desired files
elif [ -n "$TGT_DEV" ] && [ "$UserAccepts" = true ]; then
    pw $PW_DBG "Unmounting any partitions on: $TGT_DEV"
    # make sure everything is un-mounted first
    for targ in $(ls ${TGT_DEV}*); do
        pw $PW_DBG "Unmounting $targ"
        umount $targ
    done
    pw $PW_OK "$self: starting disk activities"

    ############################################################################
    # do the disk-related activities
    partition_dev  ${TGT_DEV}
    partition_load ${TGT_DEV} 10 0.25
    partition_wait ${TGT_DEV} 5
    format_dev     ${TGT_DEV}
    mount_dev      ${TGT_DEV}
    pw $PW_OK "$self: ${TGT_DEV} preparations complete."
    [[ $FORMAT_ONLY -eq 1 ]] && cleanup
    [[ $FORMAT_ONLY -eq 1 ]] && exit 0

    if [[ ! -z $TAR_FILE ]]; then
        pw $PW_NFO "Extracting image from $TAR_FILE"
        TAR_DIR=$(mktemp --tmpdir -d ZED.XXXX)
        tar -xzf $TAR_FILE -C $TAR_DIR
    fi

    pw $PW_NFO "Copying $TAR_DIR to $TGT_DEV"
    safe_cp $TAR_DIR/boot/ $MNT_BOOT/ -vrc
    safe_cp $TAR_DIR/root/ $MNT_ROOT/ -a --keep-dirlinks

    #create a swap file in case of large compiles. TODO: convert to partition
    #create_swap_file $SWAP_SZ $MNT_ROOT #Now using partition
    #echo "/dev/mmcblk0p2 none swap sw 0 0" >> $froot/etc/fstab #swap partition
    #note that swap file vs. partition requires changes here, the fdisk script,
    #the rootfs (mmcblk0p2->mmcblk0p3) in src/linux.config, the prepare_img.sh
    #(for fstab), and the mkfs (below), and the part timeout devs from 3 to 4
    [ $? -ne 0 ] && pw $PW_WRN "swap file allocation failed"

    ############################################################################
    pw $PW_OK "$self: Filesystem activities completed!"
    cleanup #sync cache and unmount
    if [ "$IMG" = true ]; then
        pw $PW_NFO "Compressing image."
        #pv $IMG_FILE | bzip2 > ${IMG_FILE}.bz2 #2:40, 652M
        pv $IMG_FILE | gzip > ${IMG_FILE}.gz    #0:53, 679M (3x faster, almost same size)
        pw $PW_NFO "Image $IMG_FILE is ready."
    else
        eject $TGT_DEV
        pw $PW_NFO "SD card $TGT_DEV is ready. It is safe to remove it."
    fi
    pw $PW_OK "$self: Done!"
else
    pw $PW_WRN "No SD card was found, or the user cancelled flashing step."
    echo "[SKLK]"
    echo "[SKLK]          This can happen if your SD card was ejected, in which"
    echo "[SKLK]          case please remove and re-insert the media."
    echo "[SKLK]"
    echo "[SKLK]          This can also happen with odd SD card readers or systems"
    echo "[SKLK]          that don't identify themselves in a currently known way."
    echo "[SKLK]"
    echo "[SKLK]          Known-working SD card readers are:"
    echo "[SKLK]            - Surface Book"
    echo "[SKLK]            - Transcend USB 3.0 SD Reader"
    echo "[SKLK]"
    echo "[SKLK]          Known-bad SD card readers are:"
    echo "[SKLK]            - Sabrent USB 3.0 Reader"
    echo "[SKLK]            - Kingston FCR-HS219/1"
    echo "[SKLK]"
    echo "[SKLK]          To add to this list, report to: info@skylarkwireless.com"
    pw $PW_ERR "Exiting"
    exit
fi #Found an SD card to Flash.
