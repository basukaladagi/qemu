/*
 * PCIe to help simulate the PCIe devices used to analyze 
 * Hardware Accelerators.
 *
 * Author: Basavaraj Kaladagi (bkaladagi@cs.stonybrook.edu)
 *
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "sysemu/kvm.h"
#include "migration/migration.h"
#include "qemu/error-report.h"
#include "qemu/event_notifier.h"
#include "qom/object_interfaces.h"
#include "sysemu/char.h"
#include "sysemu/hostmem.h"
#include "sysemu/qtest.h"
#include "qapi/visitor.h"

#include "hw/misc/ivshmem.h"
//#include "hw/misc/npu.h"
#include "npu.h"

#define PCI_VENDOR_ID_ACCELERATOR   PCI_VENDOR_ID_REDHAT_QUMRANET
#define PCI_DEVICE_ID_ACCELERATOR   0x1110

#define ACCELERATOR_MAX_PEERS UINT16_MAX
#define ACCELERATOR_IOEVENTFD   0
#define ACCELERATOR_MSI     1

#define ACCELERATOR_REG_BAR_SIZE 0x100
//#define ACCELERATOR_DATA_BAR_SIZE 0x100
#define ACCELERATOR_DATA_BAR_SIZE 512*1024*1024
#define BAR2_BEGIN 0xC0000000
#define BAR2_END

#define ACCELERATOR_DEBUG 0
#define ACCELERATOR_DPRINTF(fmt, ...)                       \
    do {                                                \
            printf("ACCELERATOR: " fmt, ## __VA_ARGS__);    \
    } while (0)

#define TYPE_ACCELERATOR_COMMON "accelerator-common"
#define ACCELERATOR_COMMON(obj) \
    OBJECT_CHECK(IVShmemState, (obj), TYPE_ACCELERATOR_COMMON)

#define TYPE_ACCELERATOR_PLAIN "accelerator-plain"
#define ACCELERATOR_PLAIN(obj) \
    OBJECT_CHECK(IVShmemState, (obj), TYPE_ACCELERATOR_PLAIN)

#define TYPE_ACCELERATOR_DOORBELL "accelerator-pcie"
#define ACCELERATOR_DOORBELL(obj) \
    OBJECT_CHECK(IVShmemState, (obj), TYPE_ACCELERATOR_DOORBELL)

#define TYPE_ACCELERATOR "accelerator"
#define ACCELERATOR(obj) \
    OBJECT_CHECK(IVShmemState, (obj), TYPE_ACCELERATOR)


#define  NPU_DATA_SIZE  sizeof(struct NpuData)

char *dummy_ptr;

char *mem_ptr;
uint64_t  test_var;

int initializaiton_count;
int connect_count;

typedef struct Peer {
    int nb_eventfds;
    EventNotifier *eventfds;
} Peer;

typedef struct MSIVector {
    PCIDevice *pdev;
    int virq;
} MSIVector;

typedef struct IVShmemState {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    uint32_t features;

    /* exactly one of these two may be set */
    HostMemoryBackend *hostmem; /* with interrupts */
    CharDriverState *server_chr; /* without interrupts */

    /* registers */
    uint32_t intrmask;
    uint32_t intrstatus;
    int vm_id;

    /* BARs */
    MemoryRegion accelerator_mmio;  /* BAR 0 (registers) */
    MemoryRegion *accelerator_bar2; /* BAR 2 (shared memory) */
    MemoryRegion server_bar2;   /* used with server_chr */
    MemoryRegion server_bar2_cont;   /* used with server_chr */

    /* interrupt support */
    Peer *peers;
    int nb_peers;               /* space in @peers[] */
    uint32_t vectors;
    MSIVector *msi_vectors;
    uint64_t msg_buf;           /* buffer for receiving server messages */
    int msg_buffered_bytes;     /* #bytes in @msg_buf */
    NpuData npu;

    /* migration stuff */
    OnOffAuto master;
    Error *migration_blocker;

    /* legacy cruft */
    char *role;
    char *shmobj;
    char *sizearg;
    size_t legacy_size;
    uint32_t not_legacy_32bit;
} IVShmemState;

/* registers for the Inter-VM shared memory device */
enum accelerator_registers {
    INTRMASK = 0,
    INTRSTATUS = 4,
    IVPOSITION = 8,
    DOORBELL = 12,
};

static inline uint32_t accelerator_has_feature(IVShmemState *ivs,
                                                    unsigned int feature) {
    return (ivs->features & (1 << feature));
}

static inline bool accelerator_is_master(IVShmemState *s)
{
    assert(s->master != ON_OFF_AUTO_AUTO);
    return s->master == ON_OFF_AUTO_ON;
}

static void accelerator_update_irq(IVShmemState *s)
{
    PCIDevice *d = PCI_DEVICE(s);
    uint32_t isr = s->intrstatus & s->intrmask;

    /*
     * Do nothing unless the device actually uses INTx.  Here's how
     * the device variants signal interrupts, what they put in PCI
     * config space:
     * Device variant    Interrupt  Interrupt Pin  MSI-X cap.
     * accelerator-plain         none            0         no
     * accelerator-doorbell     MSI-X            1        yes(1)
     * accelerator,msi=off       INTx            1         no
     * accelerator,msi=on       MSI-X            1(2)     yes(1)
     * (1) if guest enabled MSI-X
     * (2) the device lies
     * Leads to the condition for doing nothing:
     */
    if (accelerator_has_feature(s, ACCELERATOR_MSI)
        || !d->config[PCI_INTERRUPT_PIN]) {
        return;
    }

    /* don't print ISR resets */
    if (isr) {
        ACCELERATOR_DPRINTF("Set IRQ to %d (%04x %04x)\n",
                        isr ? 1 : 0, s->intrstatus, s->intrmask);
    }

    pci_set_irq(d, isr != 0);
}

static void accelerator_IntrMask_write(IVShmemState *s, uint32_t val)
{
    ACCELERATOR_DPRINTF("IntrMask write(w) val = 0x%04x\n", val);

    s->intrmask = val;
    accelerator_update_irq(s);
}

static uint32_t accelerator_IntrMask_read(IVShmemState *s)
{
    uint32_t ret = s->intrmask;

    ACCELERATOR_DPRINTF("intrmask read(w) val = 0x%04x\n", ret);
    return ret;
}

static void accelerator_IntrStatus_write(IVShmemState *s, uint32_t val)
{
    ACCELERATOR_DPRINTF("IntrStatus write(w) val = 0x%04x\n", val);

    s->intrstatus = val;
    accelerator_update_irq(s);
}

static uint32_t accelerator_IntrStatus_read(IVShmemState *s)
{
    uint32_t ret = s->intrstatus;

    /* reading ISR clears all interrupts */
    s->intrstatus = 0;
    accelerator_update_irq(s);
    return ret;
}

static void server_io_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
	IVShmemState *s = opaque;
	NpuData npu_tmp;
	char tmp[10] = "\0";
	ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);

	ACCELERATOR_DPRINTF("Address:%lu\n", addr);
	ACCELERATOR_DPRINTF("val:%ld\n", val);
	ACCELERATOR_DPRINTF("chrval:%s\n", (char *)&val);

	snprintf(tmp, sizeof(tmp), "%d", val);

	printf("tmp:%s, len:%d\n", tmp, strlen(tmp));

	memcpy((char *)mem_ptr + addr, tmp, strlen(tmp) + 1);
	//ACCELERATOR_DPRINTF("written val:%ld\n", *((uint64_t *)mem_ptr));

	memset(&npu_tmp, 0, sizeof(NpuData));
	npu_tmp.magic = 47806;
	// TODO:  enable this, when buffer is created.
	//npu_tmp.address = (uint64_t) addr;
	npu_tmp.address = (uint64_t)(BAR2_BEGIN + addr);
	npu_tmp.flags = 1;
	npu_tmp.temp_data = val;
	npu_tmp.guest_op = 1;
	npu_tmp.data_size = size;
	printf("Address sent: %lu\n",npu_tmp.address);
	/* TODO: Read data and fill it in npu_tmp */
	qemu_chr_fe_write_all(s->server_chr, (uint8_t *)&npu_tmp,
				sizeof(npu_tmp));

	ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);
	test_var = val;
	ACCELERATOR_DPRINTF("Value written: %lu\n", test_var);
	return 0;
}
static void accelerator_io_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    IVShmemState *s = opaque;

    uint16_t dest = val >> 16;
    uint16_t vector = val & 0xff;
    ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);

    addr &= 0xfc;

    ACCELERATOR_DPRINTF("writing to addr " TARGET_FMT_plx "\n", addr);
    switch (addr)
    {
        case INTRMASK:
            accelerator_IntrMask_write(s, val);
            break;

        case INTRSTATUS:
            accelerator_IntrStatus_write(s, val);
            break;

        case DOORBELL:
            /* check that dest VM ID is reasonable */
            if (dest >= s->nb_peers) {
                ACCELERATOR_DPRINTF("Invalid destination VM ID (%d)\n", dest);
                break;
            }

            /* check doorbell range */
            if (vector < s->peers[dest].nb_eventfds) {
                ACCELERATOR_DPRINTF("Notifying VM %d on vector %d\n", dest, vector);
                event_notifier_set(&s->peers[dest].eventfds[vector]);
            } else {
                ACCELERATOR_DPRINTF("Invalid destination vector %d on VM %d\n",
                                vector, dest);
            }
            break;
        default:
            ACCELERATOR_DPRINTF("Unhandled write " TARGET_FMT_plx "\n", addr);
    }
}

static uint64_t server_io_read(void *opaque, hwaddr addr,
                                unsigned size)
{
	IVShmemState *s = opaque;
	PCIDevice *dev = PCI_DEVICE(s);
	unsigned char rbuf[100];
	unsigned char rbuf2[100];
	uint64_t r1;
	uint64_t r2;
	uint64_t ret;
	NpuData npu_tmp;
	NpuData npu_test; // for receiving data
	ACCELERATOR_DPRINTF("\nin %s:%d\n", __func__, __LINE__);
	memset(rbuf, 0, 100);
	memset(rbuf2, 0, 100);
	hwaddr pa = 0x0000100;
	ACCELERATOR_DPRINTF("Before addres : r1 0x%x\t r2: 0x%x\n",
			r1, r2);
        pci_dma_read(dev, 0x0000100, (void *)&r1, 4);
	ACCELERATOR_DPRINTF("pci_dma_read result :addres data:0x%x\n",
			r1);
	if(cpu_physical_memory_is_io(pa)) {
		ACCELERATOR_DPRINTF("IO allowed on the address\n");
	} else {
		ACCELERATOR_DPRINTF("IO not allowed on the address\n");
	}
	cpu_physical_memory_read(pa, (void *) &r2, 4);
        ACCELERATOR_DPRINTF("After Read physicall addres data:0x%x\n",
			r2);
	ACCELERATOR_DPRINTF("Address sent: %lu\n",addr);
	memset(&npu_tmp, 0, sizeof(NpuData));
	//npu_tmp.address = (uint64_t) &test_var;
	npu_tmp.address = (uint64_t) (BAR2_BEGIN + addr);
	npu_tmp.magic = 47806;
	npu_tmp.flags = 1;
	npu_tmp.temp_data = test_var;
	//npu_tmp.temp_data = atoi(;
	npu_tmp.guest_op = 2;
	npu_tmp.data_size = size;

	//msi_notify(dev, 0);

	/* TODO: Read data and fill it in npu_tmp */
	qemu_chr_fe_write_all(s->server_chr, (uint8_t *)&npu_tmp,
				sizeof(npu_tmp));
	printf("after writing\n");

	memset(&npu_test,0, sizeof(NpuData));
	ACCELERATOR_DPRINTF("will be reading\n");
    	ret = qemu_chr_fe_read_all(s->server_chr, (uint8_t *)&npu_test,
                            sizeof(NpuData));
	ACCELERATOR_DPRINTF("REad data:0x%x\n", npu_test.temp_data);

	ACCELERATOR_DPRINTF("After read ret val:%d\n", ret);

	//ret = atoi(atoi(mem_ptr));
	//ret = test_var;
	ret = npu_test.temp_data;
	return ret;
}

static uint64_t accelerator_io_read(void *opaque, hwaddr addr,
                                unsigned size)
{

    IVShmemState *s = opaque;
    uint32_t ret;
    ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);

    switch (addr)
    {
        case INTRMASK:
            ret = accelerator_IntrMask_read(s);
            break;

        case INTRSTATUS:
            ret = accelerator_IntrStatus_read(s);
            break;

        case IVPOSITION:
            ret = s->vm_id;
            break;

        default:
            ACCELERATOR_DPRINTF("why are we reading " TARGET_FMT_plx "\n", addr);
            ret = 0;
    }

    return ret;
}

static const MemoryRegionOps accelerator_mmio_ops = {
    .read = accelerator_io_read,
    .write = accelerator_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps server_bar_mmio_ops = {
    .read = server_io_read,
    .write = server_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};


static void accelerator_vector_notify(void *opaque)
{
    MSIVector *entry = opaque;
    PCIDevice *pdev = entry->pdev;
    IVShmemState *s = ACCELERATOR_COMMON(pdev);
    int vector = entry - s->msi_vectors;
    EventNotifier *n = &s->peers[s->vm_id].eventfds[vector];

    if (!event_notifier_test_and_clear(n)) {
        return;
    }

    ACCELERATOR_DPRINTF("interrupt on vector %p %d\n", pdev, vector);
    if (accelerator_has_feature(s, ACCELERATOR_MSI)) {
        if (msix_enabled(pdev)) {
            msix_notify(pdev, vector);
        }
    } else {
        accelerator_IntrStatus_write(s, 1);
    }
}

static int accelerator_vector_unmask(PCIDevice *dev, unsigned vector,
                                 MSIMessage msg)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);
    EventNotifier *n = &s->peers[s->vm_id].eventfds[vector];
    MSIVector *v = &s->msi_vectors[vector];
    int ret;

    ACCELERATOR_DPRINTF("vector unmask %p %d\n", dev, vector);

    ret = kvm_irqchip_update_msi_route(kvm_state, v->virq, msg, dev);
    if (ret < 0) {
        return ret;
    }
    kvm_irqchip_commit_routes(kvm_state);

    return kvm_irqchip_add_irqfd_notifier_gsi(kvm_state, n, NULL, v->virq);
}

static void accelerator_vector_mask(PCIDevice *dev, unsigned vector)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);
    EventNotifier *n = &s->peers[s->vm_id].eventfds[vector];
    int ret;

    ACCELERATOR_DPRINTF("vector mask %p %d\n", dev, vector);

    ret = kvm_irqchip_remove_irqfd_notifier_gsi(kvm_state, n,
                                                s->msi_vectors[vector].virq);
    if (ret != 0) {
        error_report("remove_irqfd_notifier_gsi failed");
    }
}

static void accelerator_vector_poll(PCIDevice *dev,
                                unsigned int vector_start,
                                unsigned int vector_end)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);
    unsigned int vector;

    ACCELERATOR_DPRINTF("vector poll %p %d-%d\n", dev, vector_start, vector_end);

    vector_end = MIN(vector_end, s->vectors);

    for (vector = vector_start; vector < vector_end; vector++) {
        EventNotifier *notifier = &s->peers[s->vm_id].eventfds[vector];

        if (!msix_is_masked(dev, vector)) {
            continue;
        }

        if (event_notifier_test_and_clear(notifier)) {
            msix_set_pending(dev, vector);
        }
    }
}

static void watch_vector_notifier(IVShmemState *s, EventNotifier *n,
                                 int vector)
{
    int eventfd = event_notifier_get_fd(n);

    assert(!s->msi_vectors[vector].pdev);
    s->msi_vectors[vector].pdev = PCI_DEVICE(s);

    qemu_set_fd_handler(eventfd, accelerator_vector_notify,
                        NULL, &s->msi_vectors[vector]);
}

static void accelerator_add_eventfd(IVShmemState *s, int posn, int i)
{
    memory_region_add_eventfd(&s->accelerator_mmio,
                              DOORBELL,
                              4,
                              true,
                              (posn << 16) | i,
                              &s->peers[posn].eventfds[i]);
}

static void accelerator_del_eventfd(IVShmemState *s, int posn, int i)
{
    memory_region_del_eventfd(&s->accelerator_mmio,
                              DOORBELL,
                              4,
                              true,
                              (posn << 16) | i,
                              &s->peers[posn].eventfds[i]);
}

static void close_peer_eventfds(IVShmemState *s, int posn)
{
    int i, n;

    assert(posn >= 0 && posn < s->nb_peers);
    n = s->peers[posn].nb_eventfds;

    if (accelerator_has_feature(s, ACCELERATOR_IOEVENTFD)) {
        memory_region_transaction_begin();
        for (i = 0; i < n; i++) {
            accelerator_del_eventfd(s, posn, i);
        }
        memory_region_transaction_commit();
    }

    for (i = 0; i < n; i++) {
        event_notifier_cleanup(&s->peers[posn].eventfds[i]);
    }

    g_free(s->peers[posn].eventfds);
    s->peers[posn].nb_eventfds = 0;
}

static void resize_peers(IVShmemState *s, int nb_peers)
{
    int old_nb_peers = s->nb_peers;
    int i;

    assert(nb_peers > old_nb_peers);
    ACCELERATOR_DPRINTF("bumping storage to %d peers\n", nb_peers);

    s->peers = g_realloc(s->peers, nb_peers * sizeof(Peer));
    s->nb_peers = nb_peers;

    for (i = old_nb_peers; i < nb_peers; i++) {
        s->peers[i].eventfds = g_new0(EventNotifier, s->vectors);
        s->peers[i].nb_eventfds = 0;
    }
}

static void accelerator_add_kvm_msi_virq(IVShmemState *s, int vector,
                                     Error **errp)
{
    PCIDevice *pdev = PCI_DEVICE(s);
    int ret;

    ACCELERATOR_DPRINTF("accelerator_add_kvm_msi_virq vector:%d\n", vector);
    assert(!s->msi_vectors[vector].pdev);

    ret = kvm_irqchip_add_msi_route(kvm_state, vector, pdev);
    if (ret < 0) {
        error_setg(errp, "kvm_irqchip_add_msi_route failed");
        return;
    }

    s->msi_vectors[vector].virq = ret;
    s->msi_vectors[vector].pdev = pdev;
}

static void setup_interrupt(IVShmemState *s, int vector, Error **errp)
{
    EventNotifier *n = &s->peers[s->vm_id].eventfds[vector];
    bool with_irqfd = kvm_msi_via_irqfd_enabled() &&
        accelerator_has_feature(s, ACCELERATOR_MSI);
    PCIDevice *pdev = PCI_DEVICE(s);
    Error *err = NULL;

    ACCELERATOR_DPRINTF("setting up interrupt for vector: %d\n", vector);

    if (!with_irqfd) {
        ACCELERATOR_DPRINTF("with eventfd\n");
        watch_vector_notifier(s, n, vector);
    } else if (msix_enabled(pdev)) {
        ACCELERATOR_DPRINTF("with irqfd\n");
        accelerator_add_kvm_msi_virq(s, vector, &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }

        if (!msix_is_masked(pdev, vector)) {
            kvm_irqchip_add_irqfd_notifier_gsi(kvm_state, n, NULL,
                                               s->msi_vectors[vector].virq);
            /* TODO handle error */
        }
    } else {
        /* it will be delayed until msix is enabled, in write_config */
        ACCELERATOR_DPRINTF("with irqfd, delayed until msix enabled\n");
    }
}

static void process_msg_shmem(IVShmemState *s, int fd, Error **errp)
{
    struct stat buf;
    size_t size;
    void *ptr;

    initializaiton_count++;
    ACCELERATOR_DPRINTF("basu  %s:%d\n", __func__, __LINE__);
    ACCELERATOR_DPRINTF("shmem FD that was received:%d\n", fd);

    if (s->accelerator_bar2) {
        error_setg(errp, "server sent unexpected shared memory message");
        close(fd);
        return;
    }

    if (fstat(fd, &buf) < 0) {
        error_setg_errno(errp, errno,
            "can't determine size of shared memory sent by server");
        close(fd);
        return;
    }

    size = buf.st_size;
    ACCELERATOR_DPRINTF("shmsize:%ld\n", size);

    /* Legacy cruft */
    if (s->legacy_size != SIZE_MAX) {
        if (size < s->legacy_size) {
            error_setg(errp, "server sent only %zd bytes of shared memory",
                       (size_t)buf.st_size);
            close(fd);
            return;
        }
        size = s->legacy_size;
    }

    /* mmap the region and map into the BAR2 */
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        error_setg_errno(errp, errno, "Failed to mmap shared memory");
        close(fd);
        return;
    }
    memory_region_init_ram_ptr(&s->server_bar2, OBJECT(s),
                               "accelerator.bar2", size, ptr);
    memory_region_set_fd(&s->server_bar2, fd);
    s->accelerator_bar2 = &s->server_bar2;
}

static void process_msg_disconnect(IVShmemState *s, uint16_t posn,
                                   Error **errp)
{
    ACCELERATOR_DPRINTF("basu  %s:%d\n", __func__, __LINE__);
    ACCELERATOR_DPRINTF("disconneting");
    ACCELERATOR_DPRINTF("posn %d has gone away\n", posn);
    if (posn >= s->nb_peers || posn == s->vm_id) {
        error_setg(errp, "invalid peer %d", posn);
        return;
    }
    close_peer_eventfds(s, posn);
}

static void process_msg_connect(IVShmemState *s, uint16_t posn, int fd,
                                Error **errp)
{
    Peer *peer = &s->peers[posn];

    int vector;
    ACCELERATOR_DPRINTF("Creating new peer\n");
    ACCELERATOR_DPRINTF("basu  %s:%d\n", __func__, __LINE__);
    connect_count++;

    /*
     * The N-th connect message for this peer comes with the file
     * descriptor for vector N-1.  Count messages to find the vector.
     */
    if (peer->nb_eventfds >= s->vectors) {
        error_setg(errp, "Too many eventfd received, device has %d vectors",
                   s->vectors);
        close(fd);
        return;
    }
    vector = peer->nb_eventfds++;

    ACCELERATOR_DPRINTF("basu eventfds[%d][%d] = %d\n", posn, vector, fd);
    event_notifier_init_fd(&peer->eventfds[vector], fd);
    fcntl_setfl(fd, O_NONBLOCK); /* msix/irqfd poll non block */

    if (posn == s->vm_id) {
        setup_interrupt(s, vector, errp);
        /* TODO do we need to handle the error? */
    }

    if (accelerator_has_feature(s, ACCELERATOR_IOEVENTFD)) {
	ACCELERATOR_DPRINTF("before calling accelerator_add_eventfd\n");
        accelerator_add_eventfd(s, posn, vector);
    }
}

static void process_msg(IVShmemState *s, int64_t msg, int fd, Error **errp)
{

    ACCELERATOR_DPRINTF("basu  %s:%d\n", __func__, __LINE__);
    ACCELERATOR_DPRINTF("posn is %" PRId64 ", fd is %d\n", msg, fd);

    if (msg < -1 || msg > ACCELERATOR_MAX_PEERS) {
        error_setg(errp, "server sent invalid message %" PRId64, msg);
        close(fd);
        return;
    }

    if (msg == -1) {
	ACCELERATOR_DPRINTF("basu,shm msg\n");
	if(initializaiton_count) {
		ACCELERATOR_DPRINTF("initializaiton_count error\n");
	} else {
		process_msg_shmem(s, fd, errp);
	}
        return;
    }

    if (msg >= s->nb_peers) {
        resize_peers(s, msg + 1);
    }

    if (fd >= 0) {
	if(connect_count){
		ACCELERATOR_DPRINTF("Connecting again, error\n");
	} else{
		ACCELERATOR_DPRINTF("connecting :%d\n", __LINE__);
		process_msg_connect(s, msg, fd, errp);
	}
    } else {
        process_msg_disconnect(s, msg, errp);
    }
}

static int accelerator_can_receive(void *opaque)
{
//ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);
	return NPU_DATA_SIZE;
#if 0
    IVShmemState *s = opaque;
//ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);


    assert(s->msg_buffered_bytes < sizeof(s->msg_buf));
    return sizeof(s->msg_buf) - s->msg_buffered_bytes;
#endif
}

static void accelerator_read(void *opaque, const uint8_t *buf, int size)
{
    IVShmemState *s = opaque;
    PCIDevice *dev = PCI_DEVICE(s);
    NpuData *p;
    Error *err = NULL;
    int fd;
    int64_t msg;
    uint64_t r2;
    unsigned char rbuf[100] = "\0";
    char write_buf[256] = "\0";
    ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);
    ACCELERATOR_DPRINTF("Size: %d\n", size);

    p = (NpuData *) buf;
    ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);
    if (p->magic == 47806){
	    ACCELERATOR_DPRINTF("Yay, success, got the correct data\n");
	    ACCELERATOR_DPRINTF("host_req :%d\n", p->host_req);
	    ACCELERATOR_DPRINTF("addrs:%x\n", p->address);
	    ACCELERATOR_DPRINTF("data size:%d\n", p->data_size);
	    hwaddr addrtst = 0x00000100;
    		uint64_t tst;
    	    uint8_t tst2[4];
	    if(p->host_req == 2) {
		NpuData resp;
		int  k = 0;
		cpu_physical_memory_read(p->address, (void *) &r2, 4);
		cpu_physical_memory_read(addrtst, (void *) &tst, 4);
		cpu_physical_memory_read(addrtst, tst2,4); 
	
		//cpu_physical_memory_read(p->address, resp.data_test, p->data_size);
		cpu_physical_memory_read(p->address, resp.data_test,
					 p->data_size);
		ACCELERATOR_DPRINTF("readdata r2:0x%x\n", r2);
		ACCELERATOR_DPRINTF("readdata tst:0x%x\n", tst);
		ACCELERATOR_DPRINTF("readdata tst2:0x%x\n", tst2);
		ACCELERATOR_DPRINTF("-----READ DATA------\n");
		for(k = 0 ; k < p->data_size; k++) {
			ACCELERATOR_DPRINTF("0x%x\n",resp.data_test[k]); 
		}
		ACCELERATOR_DPRINTF("\n");
		resp.magic = 47806;
		resp.flags = 1;
		resp.address = p->address;
		resp.temp_data = r2;
		//resp.data_size= 8;
		resp.data_size= p->data_size; 
		qemu_chr_fe_write_all(s->server_chr, (uint8_t *)&resp,	                                sizeof(resp));
		ACCELERATOR_DPRINTF("Address from which to read:0x%x\n", 
				p->address);
		ACCELERATOR_DPRINTF("Reading phy addr:0x%x - Data: 0x%x\n",
				p->address, r2);
	    } else if(p->host_req == 1) {
		    ACCELERATOR_DPRINTF("Write data\n");
		    snprintf(write_buf, sizeof(uint64_t),
			     "%d", p->temp_data);
		   ACCELERATOR_DPRINTF("Data that willbe written:%s\n",
				   write_buf);
/*			TODO: Uncomment this when using.
		   cpu_physical_memory_write(p->address, 
				             p->data_test,
					     p->data_size);
*/
	    } else if(p->host_req == 3) {
		    ACCELERATOR_DPRINTF("MSI interrupt request in vector:%d\n",
				     p->vector);
		    //snprintf(write_buf, sizeof(uint64_t),
		//	     "%d", p->temp_data);
		    //Currently only Vector 0 supported.
/*
Write data to specific address and raise interrupt.
void msi_write_config(PCIDevice *dev, uint32_t addr, uint32_t val, int len)

msi_reset(PCIDevice *dev) : to clear the interrupts.
refer hw/pci/msi.c file for more API's.

*/
		    msi_notify(dev, 0);
		    ACCELERATOR_DPRINTF("Interrupt has been raised\n");
	    }

    }
#if 0
    if (connect_count == 0 || initializaiton_count == 0) {
	ACCELERATOR_DPRINTF("First conneciton\n");
	process_msg(s, msg, fd, &err);
    } else {
	    ACCELERATOR_DPRINTF("connection adlready done,shm iniy done");
    }
    if (err) {
        error_report_err(err);
    }
#endif
}

static int64_t accelerator_recv_msg(IVShmemState *s, int *pfd, Error **errp)
{
    int64_t msg;
    char  tmp[20];
    int n, ret;
    NpuData tmp_npu;

    n = 0;

    memset(&tmp_npu,0, sizeof(NpuData));
    ret = qemu_chr_fe_read_all(s->server_chr, (uint8_t *)&tmp_npu,
                            sizeof(NpuData));
    //ret = qemu_chr_fe_read_all(s->server_chr, (uint8_t *)tmp + n,
    //                         sizeof(tmp) - n);
    if (ret < 0 && ret != -EINTR) {
        error_setg_errno(errp, -ret, "read from server failed");
        return INT64_MIN;
    }
    //ACCELERATOR_DPRINTF("temp received: %s\n", tmp);
    ACCELERATOR_DPRINTF("IMP: Msg received: %ld\n", tmp_npu.magic);
    ACCELERATOR_DPRINTF("IMP: Msg received: %ld\n", tmp_npu.flags);
    ACCELERATOR_DPRINTF("IMP: Msg received: %p\n", tmp_npu.address);

    //msg = 10;

    *pfd = qemu_chr_fe_get_msgfd(s->server_chr);
    return msg;
}

static void accelerator_recv_setup(IVShmemState *s, Error **errp)
{
    Error *err = NULL;
    int64_t msg;
    int fd;
    ACCELERATOR_DPRINTF("in %s:%d\n", __func__, __LINE__);

    msg = accelerator_recv_msg(s, &fd, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
}

/* Select the MSI-X vectors used by device.
 * accelerator maps events to vectors statically, so
 * we just enable all vectors on init and after reset. */
static void accelerator_msix_vector_use(IVShmemState *s)
{
    PCIDevice *d = PCI_DEVICE(s);
    int i;

    for (i = 0; i < s->vectors; i++) {
        msix_vector_use(d, i);
    }
}

static void accelerator_reset(DeviceState *d)
{
    IVShmemState *s = ACCELERATOR_COMMON(d);

    s->intrstatus = 0;
    s->intrmask = 0;
    if (accelerator_has_feature(s, ACCELERATOR_MSI)) {
        accelerator_msix_vector_use(s);
    }
}

static int test_setup_interrupts(IVShmemState *s)
{
	int zz = 0;
    	PCIDevice *d = PCI_DEVICE(s);
	ACCELERATOR_DPRINTF("in %s\n", __func__);
	zz = msi_init(d, 0x100, 1, false, false, NULL);
	return zz; 
}
static int accelerator_setup_interrupts(IVShmemState *s)
{
    /* allocate QEMU callback data for receiving interrupts */
    s->msi_vectors = g_malloc0(s->vectors * sizeof(MSIVector));

    if (accelerator_has_feature(s, ACCELERATOR_MSI)) {
        if (msix_init_exclusive_bar(PCI_DEVICE(s), s->vectors, 1)) {
            return -1;
        }

        ACCELERATOR_DPRINTF("msix initialized (%d vectors)\n", s->vectors);
        accelerator_msix_vector_use(s);
    }

    return 0;
}

static void accelerator_enable_irqfd(IVShmemState *s)
{
    PCIDevice *pdev = PCI_DEVICE(s);
    int i;

    ACCELERATOR_DPRINTF("efds:%d\n", s->peers[s->vm_id].nb_eventfds);
    for (i = 0; i < s->peers[s->vm_id].nb_eventfds; i++) {
        Error *err = NULL;

        accelerator_add_kvm_msi_virq(s, i, &err);
        if (err) {
            error_report_err(err);
            /* TODO do we need to handle the error? */
        }
    }

    if (msix_set_vector_notifiers(pdev,
                                  accelerator_vector_unmask,
                                  accelerator_vector_mask,
                                  accelerator_vector_poll)) {
        error_report("accelerator: msix_set_vector_notifiers failed");
    }
}

static void accelerator_remove_kvm_msi_virq(IVShmemState *s, int vector)
{
    ACCELERATOR_DPRINTF("accelerator_remove_kvm_msi_virq vector:%d\n", vector);

    if (s->msi_vectors[vector].pdev == NULL) {
        return;
    }

    /* it was cleaned when masked in the frontend. */
    kvm_irqchip_release_virq(kvm_state, s->msi_vectors[vector].virq);

    s->msi_vectors[vector].pdev = NULL;
}

static void accelerator_disable_irqfd(IVShmemState *s)
{
    PCIDevice *pdev = PCI_DEVICE(s);
    int i;

    for (i = 0; i < s->peers[s->vm_id].nb_eventfds; i++) {
        accelerator_remove_kvm_msi_virq(s, i);
    }

    msix_unset_vector_notifiers(pdev);
}

static void accelerator_write_config(PCIDevice *pdev, uint32_t address,
                                 uint32_t val, int len)
{
    IVShmemState *s = ACCELERATOR_COMMON(pdev);
    int is_enabled, was_enabled = msix_enabled(pdev);

    pci_default_write_config(pdev, address, val, len);
    is_enabled = msix_enabled(pdev);
//    ACCELERATOR_DPRINTF("In %s msix isenabled:%d\n", __func__, is_enabled);
 //   ACCELERATOR_DPRINTF("In %s msix wasenabled:%d\n", __func__, was_enabled);

    if (kvm_msi_via_irqfd_enabled()) {
  //      ACCELERATOR_DPRINTF("In %s before irqfd enabled\n", __func__);
        if (!was_enabled && is_enabled) {
            accelerator_enable_irqfd(s);
   //         ACCELERATOR_DPRINTF("In %s irqfd enabled\n", __func__);
        } else if (was_enabled && !is_enabled) {
            accelerator_disable_irqfd(s);
        }
    }
}

static void accelerator_common_realize(PCIDevice *dev, Error **errp)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);

    Error *err = NULL;
    uint8_t *pci_conf;
    uint8_t attr = PCI_BASE_ADDRESS_SPACE_MEMORY |
        PCI_BASE_ADDRESS_MEM_PREFETCH;

    dummy_ptr = g_malloc(4);
    strncpy(dummy_ptr, "12", 4);

    printf("COntent of 0x%p : %d\n",dummy_ptr, atoi(dummy_ptr));

    /* IRQFD requires MSI */
    if (accelerator_has_feature(s, ACCELERATOR_IOEVENTFD) &&
        !accelerator_has_feature(s, ACCELERATOR_MSI)) {
        error_setg(errp, "ioeventfd/irqfd requires MSI");
        return;
    }

    pci_conf = dev->config;
    pci_conf[PCI_COMMAND] = PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
    memory_region_init_io(&s->accelerator_mmio, OBJECT(s),
			  &accelerator_mmio_ops, s,
                          "accelerator-mmio", ACCELERATOR_REG_BAR_SIZE);
#if 0
    void * mmio_ptr =  g_malloc(ACCELERATOR_DATA_BAR_SIZE);
    memory_region_init_ram_ptr(&s->server_bar2_cont, OBJECT(s), 
			       "server-mmio-container",
			       ACCELERATOR_DATA_BAR_SIZE,
			       mmio_ptr);
    vmstate_register_ram(&s->server_bar2_cont, DEVICE(s));
#endif
    memory_region_init_io(&s->server_bar2, OBJECT(s),
			  &server_bar_mmio_ops, s,
                          "server-mmio", ACCELERATOR_DATA_BAR_SIZE);
    printf("before add region\n");
   // memory_region_add_subregion(&s->server_bar2_cont, 0,
//				&s->server_bar2);
//s->server_bar2.ops = &server_bar_mmio_ops;

    ACCELERATOR_DPRINTF("address of the bar :%p\n",
		    s->server_bar2.addr);

    /* region for registers*/
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY,
                     &s->accelerator_mmio);

    ACCELERATOR_DPRINTF("registerd bar 0\n");
/* TODO: allocate memory using g_malloc(size) and use memory_API's
 * check accelerator example */
#if 0
    uint64_t sz =  1024;
    void * mmio_ptr =  g_malloc(sz);
    memory_region_init_ram_ptr(&s->add_sub, OBJECT(s), 
			       "pciemmio.bar2",sz, mmio_ptr);
    vmstate_register_ram(&s->add_sub, DEVICE(s));
    memory_region_add_subregion(&s->server_bar2, 0, &s->add_sub);

#endif
    mem_ptr = g_malloc(ACCELERATOR_DATA_BAR_SIZE);
    memset(mem_ptr, 0, ACCELERATOR_DATA_BAR_SIZE);

    attr |= PCI_BASE_ADDRESS_MEM_TYPE_64;
    //pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY,
    //                 &s->server_bar2);
    pci_register_bar(dev, 2, attr,
                     &s->server_bar2);
    ACCELERATOR_DPRINTF("registerd bar 1\n");

    if (!s->not_legacy_32bit) {
	ACCELERATOR_DPRINTF("changed to 64 bit address mem_type\n");
        attr |= PCI_BASE_ADDRESS_MEM_TYPE_64;
    }

    if (s->hostmem != NULL) {
        ACCELERATOR_DPRINTF("basu-using hostmem\n");

        s->accelerator_bar2 = host_memory_backend_get_memory(s->hostmem,
                                                         &error_abort);
    } else {
        assert(s->server_chr);
	printf("Using shared memserver\n");

        ACCELERATOR_DPRINTF("using shared memory server (socket = %s)\n",
                        s->server_chr->filename);

        /* we allocate enough space for 16 peers and grow as needed */
        //resize_peers(s, 16);

        /*
         * Receive setup messages from server synchronously.
         * Older versions did it asynchronously, but that creates a
         * number of entertaining race conditions.
         */
#if 0
	//TODO: THIS WORKS - BASU
        accelerator_recv_setup(s, &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }
#endif

	//TODO: add an fd_event callback, check its usage/
	ACCELERATOR_DPRINTF("Adding handler :%s:%d\n", __func__, __LINE__);
        qemu_chr_add_handlers(s->server_chr, accelerator_can_receive,
                              accelerator_read, NULL, s);
	ACCELERATOR_DPRINTF("after Adding handler :%s:%d\n", __func__, __LINE__);
	int zz = test_setup_interrupts(s);
	ACCELERATOR_DPRINTF("test_setup_interrupt returns:%d\n", zz);


	if (accelerator_has_feature(s, ACCELERATOR_MSI)) {
		ACCELERATOR_DPRINTF("MSI feature is present:%s:%d\n", 
				__func__, __LINE__);
	}
#if 0
        if (accelerator_setup_interrupts(s) < 0) {
            error_setg(errp, "failed to initialize interrupts");
            return;
        }
#endif
    }

    ACCELERATOR_DPRINTF("basu-registerd bar2");
#if 0
    vmstate_register_ram(s->accelerator_bar2, DEVICE(s));
    //TODO: create one more mmio ops structure and pass it here.
    //qemu_chr_fe_write can be used to write data to char device
    //a server(on host side) should then read the data.
    pci_register_bar(PCI_DEVICE(s), 2, attr, s->accelerator_bar2);

    if (s->master == ON_OFF_AUTO_AUTO) {
        s->master = s->vm_id == 0 ? ON_OFF_AUTO_ON : ON_OFF_AUTO_OFF;
    }

    if (!accelerator_is_master(s)) {
        error_setg(&s->migration_blocker,
                   "Migration is disabled when using feature 'peer mode' in device 'accelerator'");
        migrate_add_blocker(s->migration_blocker);
    }
#endif
}

static void accelerator_exit(PCIDevice *dev)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);
    int i;

    if (s->migration_blocker) {
        migrate_del_blocker(s->migration_blocker);
        error_free(s->migration_blocker);
    }

    if (memory_region_is_mapped(s->accelerator_bar2)) {
        if (!s->hostmem) {
            void *addr = memory_region_get_ram_ptr(s->accelerator_bar2);
            int fd;

            if (munmap(addr, memory_region_size(s->accelerator_bar2) == -1)) {
                error_report("Failed to munmap shared memory %s",
                             strerror(errno));
            }

            fd = memory_region_get_fd(s->accelerator_bar2);
            close(fd);
        }

        vmstate_unregister_ram(s->accelerator_bar2, DEVICE(dev));
    }

    if (s->peers) {
        for (i = 0; i < s->nb_peers; i++) {
            close_peer_eventfds(s, i);
        }
        g_free(s->peers);
    }

    if (accelerator_has_feature(s, ACCELERATOR_MSI)) {
        msix_uninit_exclusive_bar(dev);
    }

    g_free(s->msi_vectors);
}

static int accelerator_pre_load(void *opaque)
{
    IVShmemState *s = opaque;

    if (!accelerator_is_master(s)) {
        error_report("'peer' devices are not migratable");
        return -EINVAL;
    }

    return 0;
}

static int accelerator_post_load(void *opaque, int version_id)
{
    IVShmemState *s = opaque;

    if (accelerator_has_feature(s, ACCELERATOR_MSI)) {
        accelerator_msix_vector_use(s);
    }
    return 0;
}

static void accelerator_common_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = accelerator_common_realize;
    k->exit = accelerator_exit;
    k->config_write = accelerator_write_config;
    k->vendor_id = PCI_VENDOR_ID_ACCELERATOR;
    k->device_id = PCI_DEVICE_ID_ACCELERATOR;
    k->class_id = PCI_CLASS_MEMORY_RAM;
    k->revision = 1;
    dc->reset = accelerator_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "NPU-HOST-VM-Data transfer";
}

static const TypeInfo accelerator_common_info = {
    .name          = TYPE_ACCELERATOR_COMMON,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IVShmemState),
    .abstract      = true,
    .class_init    = accelerator_common_class_init,
};

static void accelerator_check_memdev_is_busy(Object *obj, const char *name,
                                         Object *val, Error **errp)
{
    if (host_memory_backend_is_mapped(MEMORY_BACKEND(val))) {
        char *path = object_get_canonical_path_component(val);
        error_setg(errp, "can't use already busy memdev: %s", path);
        g_free(path);
    } else {
        qdev_prop_allow_set_link_before_realize(obj, name, val, errp);
    }
}

static const VMStateDescription accelerator_plain_vmsd = {
    .name = TYPE_ACCELERATOR_PLAIN,
    .version_id = 0,
    .minimum_version_id = 0,
    .pre_load = accelerator_pre_load,
    .post_load = accelerator_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, IVShmemState),
        VMSTATE_UINT32(intrstatus, IVShmemState),
        VMSTATE_UINT32(intrmask, IVShmemState),
        VMSTATE_END_OF_LIST()
    },
};

static Property accelerator_plain_properties[] = {
    DEFINE_PROP_ON_OFF_AUTO("master", IVShmemState, master, ON_OFF_AUTO_OFF),
    DEFINE_PROP_END_OF_LIST(),
};

static void accelerator_plain_init(Object *obj)
{
    IVShmemState *s = ACCELERATOR_PLAIN(obj);

    object_property_add_link(obj, "memdev", TYPE_MEMORY_BACKEND,
                             (Object **)&s->hostmem,
                             accelerator_check_memdev_is_busy,
                             OBJ_PROP_LINK_UNREF_ON_RELEASE,
                             &error_abort);
}

static void accelerator_plain_realize(PCIDevice *dev, Error **errp)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);
    ACCELERATOR_DPRINTF("IN %s:%d\n", __func__, __LINE__);

    if (!s->hostmem) {
        error_setg(errp, "You must specify a 'memdev'");
        return;
    }

    accelerator_common_realize(dev, errp);
    host_memory_backend_set_mapped(s->hostmem, true);
}

static void accelerator_plain_exit(PCIDevice *pci_dev)
{
    IVShmemState *s = ACCELERATOR_COMMON(pci_dev);

    host_memory_backend_set_mapped(s->hostmem, false);
}

static void accelerator_plain_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = accelerator_plain_realize;
    k->exit = accelerator_plain_exit;
    dc->props = accelerator_plain_properties;
    dc->vmsd = &accelerator_plain_vmsd;
}

static const TypeInfo accelerator_plain_info = {
    .name          = TYPE_ACCELERATOR_PLAIN,
    .parent        = TYPE_ACCELERATOR_COMMON,
    .instance_size = sizeof(IVShmemState),
    .instance_init = accelerator_plain_init,
    .class_init    = accelerator_plain_class_init,
};

static const VMStateDescription accelerator_doorbell_vmsd = {
    .name = TYPE_ACCELERATOR_DOORBELL,
    .version_id = 0,
    .minimum_version_id = 0,
    .pre_load = accelerator_pre_load,
    .post_load = accelerator_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, IVShmemState),
        VMSTATE_MSIX(parent_obj, IVShmemState),
        VMSTATE_UINT32(intrstatus, IVShmemState),
        VMSTATE_UINT32(intrmask, IVShmemState),
        VMSTATE_END_OF_LIST()
    },
};

static Property accelerator_doorbell_properties[] = {
    DEFINE_PROP_CHR("chardev", IVShmemState, server_chr),
    DEFINE_PROP_UINT32("vectors", IVShmemState, vectors, 1),
    DEFINE_PROP_BIT("ioeventfd", IVShmemState, features, ACCELERATOR_IOEVENTFD,
                    true),
    DEFINE_PROP_ON_OFF_AUTO("master", IVShmemState, master, ON_OFF_AUTO_OFF),
    DEFINE_PROP_END_OF_LIST(),
};

static void accelerator_doorbell_init(Object *obj)
{
    IVShmemState *s = ACCELERATOR_DOORBELL(obj);
    ACCELERATOR_DPRINTF("Doorbel init\n");

    s->features |= (1 << ACCELERATOR_MSI);
    s->legacy_size = SIZE_MAX;  /* whatever the server sends */
}

static void accelerator_doorbell_realize(PCIDevice *dev, Error **errp)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);
    ACCELERATOR_DPRINTF("Doorbel realize\n");

    if (!s->server_chr) {
        error_setg(errp, "You must specify a 'chardev'");
        return;
    }

    accelerator_common_realize(dev, errp);
}

static void accelerator_doorbell_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    ACCELERATOR_DPRINTF("Doorbel classinit\n");
    k->realize = accelerator_doorbell_realize;
    dc->props = accelerator_doorbell_properties;
    dc->vmsd = &accelerator_doorbell_vmsd;
}

static const TypeInfo accelerator_doorbell_info = {
    .name          = TYPE_ACCELERATOR_DOORBELL,
    .parent        = TYPE_ACCELERATOR_COMMON,
    .instance_size = sizeof(IVShmemState),
    .instance_init = accelerator_doorbell_init,
    .class_init    = accelerator_doorbell_class_init,
};

static int accelerator_load_old(QEMUFile *f, void *opaque, int version_id)
{
    IVShmemState *s = opaque;
    PCIDevice *pdev = PCI_DEVICE(s);
    int ret;

    ACCELERATOR_DPRINTF("accelerator_load_old\n");

    if (version_id != 0) {
        return -EINVAL;
    }

    ret = accelerator_pre_load(s);
    if (ret) {
        return ret;
    }

    ret = pci_device_load(pdev, f);
    if (ret) {
        return ret;
    }

    if (accelerator_has_feature(s, ACCELERATOR_MSI)) {
        msix_load(pdev, f);
        accelerator_msix_vector_use(s);
    } else {
        s->intrstatus = qemu_get_be32(f);
        s->intrmask = qemu_get_be32(f);
    }

    return 0;
}

static bool test_msix(void *opaque, int version_id)
{
    IVShmemState *s = opaque;

    return accelerator_has_feature(s, ACCELERATOR_MSI);
}

static bool test_no_msix(void *opaque, int version_id)
{
    return !test_msix(opaque, version_id);
}

static const VMStateDescription accelerator_vmsd = {
    .name = "accelerator",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_load = accelerator_pre_load,
    .post_load = accelerator_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, IVShmemState),

        VMSTATE_MSIX_TEST(parent_obj, IVShmemState, test_msix),
        VMSTATE_UINT32_TEST(intrstatus, IVShmemState, test_no_msix),
        VMSTATE_UINT32_TEST(intrmask, IVShmemState, test_no_msix),

        VMSTATE_END_OF_LIST()
    },
    .load_state_old = accelerator_load_old,
    .minimum_version_id_old = 0
};

static Property accelerator_properties[] = {
    DEFINE_PROP_CHR("chardev", IVShmemState, server_chr),
    DEFINE_PROP_STRING("size", IVShmemState, sizearg),
    DEFINE_PROP_UINT32("vectors", IVShmemState, vectors, 1),
    DEFINE_PROP_BIT("ioeventfd", IVShmemState, features, ACCELERATOR_IOEVENTFD,
                    false),
    DEFINE_PROP_BIT("msi", IVShmemState, features, ACCELERATOR_MSI, true),
    DEFINE_PROP_STRING("shm", IVShmemState, shmobj),
    DEFINE_PROP_STRING("role", IVShmemState, role),
    DEFINE_PROP_UINT32("use64", IVShmemState, not_legacy_32bit, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void desugar_shm(IVShmemState *s)
{
    Object *obj;
    char *path;

    obj = object_new("memory-backend-file");
    path = g_strdup_printf("/dev/shm/%s", s->shmobj);
    object_property_set_str(obj, path, "mem-path", &error_abort);
    g_free(path);
    object_property_set_int(obj, s->legacy_size, "size", &error_abort);
    object_property_set_bool(obj, true, "share", &error_abort);
    object_property_add_child(OBJECT(s), "internal-shm-backend", obj,
                              &error_abort);
    user_creatable_complete(obj, &error_abort);
    s->hostmem = MEMORY_BACKEND(obj);
}

static void accelerator_realize(PCIDevice *dev, Error **errp)
{
    IVShmemState *s = ACCELERATOR_COMMON(dev);

    if (!qtest_enabled()) {
        error_report("accelerator is deprecated, please use accelerator-plain"
                     " or accelerator-doorbell instead");
    }

    if (!!s->server_chr + !!s->shmobj != 1) {
        error_setg(errp, "You must specify either 'shm' or 'chardev'");
        return;
    }

    if (s->sizearg == NULL) {
        s->legacy_size = 4 << 20; /* 4 MB default */
    } else {
        char *end;
        int64_t size = qemu_strtosz(s->sizearg, &end);
        if (size < 0 || (size_t)size != size || *end != '\0'
            || !is_power_of_2(size)) {
            error_setg(errp, "Invalid size %s", s->sizearg);
            return;
        }
        s->legacy_size = size;
    }

    /* check that role is reasonable */
    if (s->role) {
        if (strncmp(s->role, "peer", 5) == 0) {
            s->master = ON_OFF_AUTO_OFF;
        } else if (strncmp(s->role, "master", 7) == 0) {
            s->master = ON_OFF_AUTO_ON;
        } else {
            error_setg(errp, "'role' must be 'peer' or 'master'");
            return;
        }
    } else {
        s->master = ON_OFF_AUTO_AUTO;
    }

    if (s->shmobj) {
        desugar_shm(s);
    }

    /*
     * Note: we don't use INTx with ACCELERATOR_MSI at all, so this is a
     * bald-faced lie then.  But it's a backwards compatible lie.
     */
    pci_config_set_interrupt_pin(dev->config, 1);

    accelerator_common_realize(dev, errp);
}

static void accelerator_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = accelerator_realize;
    k->revision = 0;
    dc->desc = "Accelerator-PCIe.";
    dc->props = accelerator_properties;
    dc->vmsd = &accelerator_vmsd;
}

static const TypeInfo accelerator_info = {
    .name          = TYPE_ACCELERATOR,
    .parent        = TYPE_ACCELERATOR_COMMON,
    .instance_size = sizeof(IVShmemState),
    .class_init    = accelerator_class_init,
};

static void accelerator_register_types(void)
{
    type_register_static(&accelerator_common_info);
    type_register_static(&accelerator_plain_info);
    type_register_static(&accelerator_doorbell_info);
    type_register_static(&accelerator_info);
}

type_init(accelerator_register_types)
