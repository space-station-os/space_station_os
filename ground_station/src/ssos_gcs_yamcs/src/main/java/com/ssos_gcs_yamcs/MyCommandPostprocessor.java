package com.ssos_gcs_yamcs;

import org.yamcs.YConfiguration;
import org.yamcs.cmdhistory.CommandHistoryPublisher;
import org.yamcs.commanding.PreparedCommand;
import org.yamcs.tctm.CcsdsSeqCountFiller;
import org.yamcs.tctm.CommandPostprocessor;
import org.yamcs.utils.ByteArrayUtils;

/**
 * Component capable of modifying command binary before passing it to the link for further dispatch.
 * <p>
 * A single instance of this class is created, scoped to the link udp-out.
 * <p>
 * This is specified in the configuration file yamcs.ssos_gcs_yamcs.yaml:
 * 
 * <pre>
 * ...
 * dataLinks:
 *   - name: udp-out
 *     class: org.yamcs.tctm.UdpTcDataLink
 *     stream: tc_realtime
 *     host: localhost
 *     port: 10025
 *     commandPostprocessorClassName: com.ssos_gcs_yamcs.MyCommandPostprocessor
 * ...
 * </pre>
 */
public class MyCommandPostprocessor implements CommandPostprocessor {

    private CcsdsSeqCountFiller seqFiller = new CcsdsSeqCountFiller();
    private CommandHistoryPublisher commandHistory;

    // Constructor used when this postprocessor is used without YAML configuration
    public MyCommandPostprocessor(String yamcsInstance) {
        this(yamcsInstance, YConfiguration.emptyConfig());
    }

    // Constructor used when this postprocessor is used with YAML configuration
    // (commandPostprocessorClassArgs)
    public MyCommandPostprocessor(String yamcsInstance, YConfiguration config) {
    }

    // Called by Yamcs during initialization
    @Override
    public void setCommandHistoryPublisher(CommandHistoryPublisher commandHistory) {
        this.commandHistory = commandHistory;
    }

    // Called by Yamcs *after* a command was submitted, but *before* the link handles it.
    // This method must return the (possibly modified) packet binary.
    @Override
    public byte[] process(PreparedCommand pc) {
        byte[] binary = pc.getBinary();

        // Set CCSDS packet length
        ByteArrayUtils.encodeUnsignedShort(binary.length - 7, binary, 4);

        // Set CCSDS sequence count
        int seqCount = seqFiller.fill(binary);

        // Publish the sequence count to Command History. This has no special
        // meaning to Yamcs, but it shows how to store custom information specific
        // to a command.
        commandHistory.publish(pc.getCommandId(), "ccsds-seqcount", seqCount);

        // Since we modified the binary, update the binary in Command History too.
        commandHistory.publish(pc.getCommandId(), PreparedCommand.CNAME_BINARY, binary);

        return binary;
    }
}
