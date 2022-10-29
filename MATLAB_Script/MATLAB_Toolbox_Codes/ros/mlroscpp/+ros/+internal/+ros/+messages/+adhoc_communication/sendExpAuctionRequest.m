function [data, info] = sendExpAuctionRequest
%SendExpAuction gives an empty data for adhoc_communication/SendExpAuctionRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendExpAuctionRequest';
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Auction, info.Auction] = ros.internal.ros.messages.adhoc_communication.expAuction;
info.Auction.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendExpAuctionRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
info.MatPath{1} = 'topic';
info.MatPath{2} = 'dst_robot';
info.MatPath{3} = 'auction';
info.MatPath{4} = 'auction.auction_status_message';
info.MatPath{5} = 'auction.start_auction';
info.MatPath{6} = 'auction.auction_finished';
info.MatPath{7} = 'auction.occupied_ids';
info.MatPath{8} = 'auction.occupied_ids.id';
info.MatPath{9} = 'auction.occupied_ids.detected_by_robot_str';
info.MatPath{10} = 'auction.requested_clusters';
info.MatPath{11} = 'auction.requested_clusters.ids_contained';
info.MatPath{12} = 'auction.requested_clusters.ids_contained.id';
info.MatPath{13} = 'auction.requested_clusters.ids_contained.detected_by_robot_str';
info.MatPath{14} = 'auction.requested_clusters.bid';
info.MatPath{15} = 'auction.auction_id';
info.MatPath{16} = 'auction.robot_name';
info.MatPath{17} = 'auction.available_clusters';
info.MatPath{18} = 'auction.available_clusters.ids_contained';
info.MatPath{19} = 'auction.available_clusters.ids_contained.id';
info.MatPath{20} = 'auction.available_clusters.ids_contained.detected_by_robot_str';
info.MatPath{21} = 'auction.available_clusters.bid';
